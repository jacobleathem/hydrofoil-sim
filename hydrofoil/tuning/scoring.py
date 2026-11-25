"""
Score a parameter set by flying it through a few standard scenarios.

Each scenario returns plain metrics (no weighting); `cost` folds them into one number for the autotuner. The
scenarios scale with the boat (wave height from the flying height, wavelength from the hull length, CG shift from
the hull length), so the same suite works for the 25 kg model and the manned preset.

    takeoff    from rest in calm water: time to foilborne, 2 cm settle time, overshoot, steady error
    head       flying start, head sea:  height rms, peak, flap rate rms
    following  flying start, following sea (the hard case for the height estimate)
    shift      flying start, CG jumps forward and to the side (a rider moving): height and roll peaks
    noisy      head sea with the noisy sensors and estimators (what the boat would really measure), so a tune
               cannot win by amplifying sensor noise into the flaps
    turn       flying start, 12 deg banked 3D turn via split front flaps and back down: roll tracking, overshoot,
               height holding, and sideslip
    landing    flying start on foil, descends back onto the hull and slows down: touchdown sink rate, attitude,
               post-landing bobbing, and clean LQR -> PID handback
    supervised the takeoff as the boat really flies it: through the flight-mode supervisor (control/supervisor.py),
               displacement at 3 m/s, foil request at 3 s, until FOILING and 10 s beyond. Fails if the first
               takeoff does not reach FOILING (an abort and retry is a failure: e.g. the takeoff timeout when a stiff
               heave loop holding the hull at its reference digs it in on the drag hump) or the hull touches down
               after liftoff. Only for presets with `supervisor_tuning` (the
               supervisor's speed envelope is sized for the manned boat); see `scenarios_for`.

Scenarios are also grouped by flight phase in `PHASE_SCENARIOS` (`rising`, `cruising`, `falling`) so phase-aware
autotuning can optimise PID loops on rising/falling and LQR weights on foilborne cruising and 3D turning.

Hard failures (hull touches down after liftoff in non-landing runs, excessive roll/pitch, non-finite state) set `failed`.
"""

from __future__ import annotations

import warnings

import numpy as np

SCENARIOS = ("takeoff", "head", "following", "shift", "noisy")
PHASE_SCENARIOS = {
    "rising": ("takeoff", "supervised"),
    "cruising": ("head", "following", "shift", "noisy", "turn"),
    "falling": ("landing",),
}
SUPERVISED_PLAN = ((0.0, {"speed": 3.0}), (3.0, {"foil": True, "speed": 7.0}))
SUPERVISED_T = 45.0                                  # s: 25 s takeoff timeout after the request, plus margin


def scenarios_for(params: dict) -> tuple:
    """The scenario list for a preset: SCENARIOS, plus the supervised takeoff where the supervisor applies."""
    return SCENARIOS + (("supervised",) if params.get("supervisor_tuning") else ())


def phase_scenarios(params: dict, phase: str | None = None) -> tuple[str, ...]:
    """Scenarios grouped by flight phase ('rising', 'cruising', 'falling', or all combined when phase is None)."""
    sup = bool(params.get("supervisor_tuning"))
    phases = ("rising", "cruising", "falling") if phase is None else (phase,)
    out = []
    for ph in phases:
        for name in PHASE_SCENARIOS[ph]:
            if name == "supervised" and not sup:
                continue
            out.append(name)
    return tuple(out)


def _run_supervised(params: dict) -> dict:
    from hydrofoil.vehicle import VehiclePhysics
    from hydrofoil.waves.airy import CalmWater
    p = dict(params, supervisor=True)
    dt = p["dt"]
    try:
        vp = VehiclePhysics(p)
    except Exception as e:
        return {"failed": True, "why": f"build: {e}"}
    sup = vp.supervisor
    s, t = vp.rest_state(), 0.0
    t_req, t_foil, lifted, touch = SUPERVISED_PLAN[1][0], None, False, 0
    v_takeoff_cmd = max(7.0, float(p.get("target_speed", 7.0)), float(p.get("lqr_engage_speed", 7.0)) + 0.5)
    rec = []
    for i in range(int(SUPERVISED_T / dt)):
        for tt, cmd in SUPERVISED_PLAN:
            if i == int(round(tt / dt)):
                cmd_use = dict(cmd)
                if cmd_use.get("foil") and "speed" in cmd_use:
                    cmd_use["speed"] = v_takeoff_cmd
                sup.pilot.update(cmd_use)
        s, c, th, draft, _ = vp.step(s, t, CalmWater())
        t += dt
        if not np.all(np.isfinite(s)):
            return {"failed": True, "why": "non-finite state"}
        if abs(np.degrees(s[3])) > 30.0:
            return {"failed": True, "why": "capsize (|roll| > 30 deg)"}
        if draft <= 1e-6:
            lifted = True
        elif lifted:
            touch += 1
        if t_foil is None and any(frm == "TAKEOFF" and to != "FOILING" for _t, frm, to, _w in sup.log):
            why = next(w for _t, frm, to, w in sup.log if frm == "TAKEOFF" and to != "FOILING")
            return {"failed": True, "why": f"supervised takeoff aborted: {why}"}       # a retry is not a pass
        if sup.mode == "FOILING" and t_foil is None:
            t_foil = t
        if t_foil is not None:
            zref = vp.hull_offset - p["flying_height"]          # the supervisor moves the reference
            lim = any(v.get("limited") for v in c.get("drive", {}).values())
            rec.append((t, s[2] - zref, np.degrees(s[3]), np.degrees(s[4]), c["delta_main"], lim))
        if t_foil is not None and t > t_foil + 10.0:
            break
    if t_foil is None:
        why = next((w for _t, _f, to, w in sup.log if to == "LANDING"), "FOILING not reached")
        return {"failed": True, "why": f"supervised takeoff: {why}"}
    r = np.array(rec, dtype=float)
    tt, dz, roll, pitch, fm = r[:, 0], r[:, 1], r[:, 2], r[:, 3], r[:, 4]
    m = tt > t_foil + 3.0
    out = {"failed": touch > 0, "t_foiling": float(t_foil - t_req),
           "z_rms": float(dz[m].std()), "z_peak": float(np.abs(dz[m]).max()),
           "roll_peak": float(np.abs(roll[m]).max()), "pitch_peak": float(np.abs(pitch[m] - pitch[m].mean()).max()),
           "flap_rate_rms": float(np.gradient(fm[m], dt).std()), "sat_frac": float(np.mean(np.abs(fm[m]) > 19.0)),
           "limited_frac": float(r[m, 5].mean())}
    if touch:
        out["why"] = "touched down after liftoff (supervised)"
    return out


def _flying_state(vp):
    from hydrofoil.linearize import trim
    p = vp.params
    try:
        x, u0 = trim(vp, p["target_speed"])
    except Exception:                                    # no trim at this speed: start at rest instead
        return vp.rest_state()
    if vp.lqr is not None and p.get("controller") in ("lqr", "auto"):
        vp.lqr.engaged = True
        vp.lqr.since_engage = vp.lqr.blend_s + 1.0
        vp.lqr.last[:] = u0[:3]
        vp.lqr.hist[:] = u0[:3]
    return x


def run_scenario(params: dict, name: str) -> dict:
    """Fly one scenario and return its metrics."""
    warnings.simplefilter("ignore")
    if name == "supervised":
        return _run_supervised(params)
    from hydrofoil.dynamics.kinematics import J
    from hydrofoil.vehicle import VehiclePhysics
    from hydrofoil.waves.airy import AiryWave, CalmWater
    p = dict(params)
    dt = p["dt"]
    L = p["hull_length"]
    amp = 0.3 * p["flying_height"]
    lam = 3.0 * L
    h0_init = float(p["flying_height"])
    v0_init = float(p["target_speed"])
    if name == "takeoff":
        T, wave, start, settle_from = 22.0, CalmWater(), "rest", 0.0
    elif name in ("head", "following", "noisy"):
        T, start, settle_from = 14.0, "flying", 3.0
        wave = AiryWave(amp, lam, direction=0.0 if name == "following" else np.pi)
        if name == "noisy":
            p["sensors"] = "noisy"
    elif name == "shift":
        T, wave, start, settle_from = 14.0, CalmWater(), "flying", 3.0
    elif name == "turn":
        T, wave, start, settle_from = 10.0, CalmWater(), "flying", 0.5
    elif name == "landing":
        T, wave, start, settle_from = 8.0, CalmWater(), "flying", 0.5
    else:
        raise ValueError(name)
    try:
        vp = VehiclePhysics(p)
    except Exception as e:                               # e.g. no trim for the LQR design
        return {"failed": True, "why": f"build: {e}"}
    s = vp.rest_state() if start == "rest" else _flying_state(vp)
    zref = vp.hull_offset - p["flying_height"]
    t, rec = 0.0, []
    shift_at = 5.0
    touch = None
    phi_tgt_max = 12.0
    if name == "turn":
        from hydrofoil.control.supervisor import compute_turn_envelope
        te0 = compute_turn_envelope(vp, s, wave_amp=0.0)
        phi_tgt_max = float(np.clip(0.85 * float(te0.get("bank_max_deg", 12.0)), 4.0, 12.0))
    for i in range(int(T / dt)):
        if name == "shift" and abs(t - shift_at) < dt / 2:
            vp.set_cg_offset((0.05 * L, 0.03 * L))
        elif name == "turn":
            if 1.0 <= t < 2.0:
                phi_cmd = min(phi_tgt_max, (t - 1.0) * phi_tgt_max)
            elif 2.0 <= t < 7.0:
                phi_cmd = phi_tgt_max
            elif 7.0 <= t < 8.0:
                phi_cmd = max(0.0, phi_tgt_max - (t - 7.0) * phi_tgt_max)
            else:
                phi_cmd = 0.0
            vp.params["phi_ref"] = phi_cmd
            if vp.supervisor is not None:
                vp.supervisor.pilot["bank"] = phi_cmd
        elif name == "landing" and t >= 1.0:
            vp.params["flying_height"] = max(-0.10, h0_init - 0.25 * (t - 1.0))
            vp.params["target_speed"] = max(2.0, 0.40 * v0_init)
            if vp.supervisor is not None:
                vp.supervisor.pilot["foil"] = False
        s, c, th, draft, _ = vp.step(s, t, wave)
        t += dt
        if not np.all(np.isfinite(s)):
            return {"failed": True, "why": "non-finite state"}
        lim = any(v.get("limited") for v in c.get("drive", {}).values())
        phi_ref_now = float(vp.params["phi_ref"])
        beta_deg = abs(float(np.degrees(np.arctan2(s[7], max(abs(s[6]), 0.1)))))
        if name == "landing":
            vz_ned = float((J(s[:6]) @ s[6:])[2])
            if t >= 1.0 and touch is None and draft > 0.005:
                touch = (max(0.0, vz_ned), abs(float(np.degrees(s[4]))), abs(float(np.degrees(s[3]))), float(t))
            if abs(np.degrees(s[3])) > 45.0 or abs(np.degrees(s[4])) > 35.0:
                return {"failed": True, "why": "landing crash (attitude exceeded limit)"}
        elif abs(np.degrees(s[3])) > 30.0:
            return {"failed": True, "why": "capsize (|roll| > 30 deg)"}
        rec.append((t, s[2] - zref, np.degrees(s[3]), np.degrees(s[4]), draft, c["delta_main"], c["delta_elev"],
                    c["delta_roll"], lim, phi_ref_now, beta_deg, s[2]))
    r = np.array(rec, dtype=float)
    tt, dz, roll, pitch, draft, fm = r[:, 0], r[:, 1], r[:, 2], r[:, 3], r[:, 4], r[:, 5]
    out = {"failed": False}
    if name == "takeoff":
        air = np.where(draft <= 1e-6)[0]
        if not len(air):
            return {"failed": True, "why": "never foilborne"}
        i0 = air[0]
        out["t_foilborne"] = float(tt[i0])
        if draft[i0:].max() > 0:
            out.update(failed=True, why="touched down after liftoff")
        ok = np.abs(dz) < 0.02
        settle = next((tt[i] for i in range(len(tt)) if ok[i:].all()), np.nan)
        out["t_settle"] = float(settle) if np.isfinite(settle) else T + 10.0
        out["overshoot"] = float(max(0.0, -dz[i0:].min()))           # dz < 0: above the target
        out["steady_err"] = float(abs(dz[tt > T - 4].mean()))
    m = tt > settle_from
    if name == "landing":
        if touch is None:
            vz_touch, touch_pitch, touch_roll, t_touch = 0.8, float(abs(pitch[-1])), float(abs(roll[-1])), T
        else:
            vz_touch, touch_pitch, touch_roll, t_touch = touch
        if vz_touch > 1.2:
            return {"failed": True, "why": f"hard landing slam (vz={vz_touch:.2f} m/s)"}
        out["vz_touch"] = float(vz_touch)
        out["touch_pitch"] = float(touch_pitch)
        out["touch_roll"] = float(touch_roll)
        out["t_touch"] = float(t_touch)
        out["handback_ok"] = bool(not vp.lqr.engaged) if (p.get("controller") == "auto" and vp.lqr is not None) else True
        late = tt > max(T - 2.5, t_touch if touch is not None else T - 2.5)
        if not late.any():
            late = m
        z_late = r[late, 11] - r[late, 11].mean()
        out["z_rms"] = float(z_late.std())
        out["z_peak"] = float(np.abs(z_late).max())
    else:
        out["z_rms"] = float(dz[m].std() if name != "takeoff" else dz[tt > T - 6].std())
        out["z_peak"] = float(np.abs(dz[m]).max())
    if name == "turn":
        bank_win = (tt >= 1.0) & (tt < 7.0)
        hold_win = (tt >= 2.5) & (tt < 7.0)
        out["roll_overshoot"] = float(max(0.0, float(roll[bank_win].max()) - phi_tgt_max))
        out["roll_rms_err"] = float(np.sqrt(np.mean((roll[hold_win] - r[hold_win, 9]) ** 2)))
        out["sideslip_peak"] = float(r[m, 10].max())
    out["roll_peak"] = float(np.abs(roll[m]).max())
    out["pitch_peak"] = float(np.abs(pitch[m] - pitch[m].mean()).max())
    out["flap_rate_rms"] = float(np.gradient(fm[m], dt).std())
    out["sat_frac"] = float(np.mean(np.abs(fm[m]) > 19.0))
    out["limited_frac"] = float(r[m, 8].mean())
    if name not in ("takeoff", "landing") and draft[m].max() > 0:
        out.update(failed=True, why="hull touched the water")
    return out


def cost(metrics: dict, scale: float) -> float:
    """One number, lower is better. `scale` is the flying height [m], so height errors count relative to it."""
    c = 0.0
    for name, m in metrics.items():
        if m.get("failed"):
            c += 100.0
            continue
        if name == "landing":
            c += 2.5 * (m.get("vz_touch", 0.3) / 0.25) ** 2 + 0.35 * m.get("touch_pitch", 0.0) + 0.35 * m.get("touch_roll", 0.0)
            c += 1.5 * m["z_rms"] / (0.05 * scale)
            c += 0.01 * m["flap_rate_rms"] + 5.0 * m["sat_frac"] + 20.0 * m["limited_frac"] + 60.0 * max(0.0, m["limited_frac"] - 0.1)
            if not m.get("handback_ok", True):
                c += 2.5
            continue
        c += 2.0 * m["z_rms"] / (0.05 * scale) + 0.5 * m["z_peak"] / (0.2 * scale)
        # drive effort: torque/voltage limiting means heat, wear and no authority left for the next gust, so it is
        # charged steeply (the first PID tune held the drives at their limit 63 % of the time on noisy sensors)
        c += 0.01 * m["flap_rate_rms"] + 5.0 * m["sat_frac"] + 20.0 * m["limited_frac"] + 60.0 * max(0.0, m["limited_frac"] - 0.1)
        if name == "takeoff":
            c += 0.08 * m["t_settle"] + 1.0 * m["overshoot"] / (0.1 * scale) + 4.0 * m["steady_err"] / (0.05 * scale)
        if name == "shift":
            c += 0.5 * m["roll_peak"]
        if name == "turn":
            c += 0.8 * m.get("roll_rms_err", 0.0) + 1.0 * m.get("roll_overshoot", 0.0) + 0.4 * m.get("sideslip_peak", 0.0)
        if name == "supervised":
            c += 0.08 * m["t_foiling"]
    return float(c)


def phase_costs(metrics: dict, scale: float) -> dict[str, float]:
    """Per-flight-phase cost breakdown ('rising', 'cruising', 'falling', plus 'total')."""
    out = {}
    for ph, names in PHASE_SCENARIOS.items():
        sub = {n: metrics[n] for n in names if n in metrics}
        if sub:
            out[ph] = cost(sub, scale)
    known = {n for names in PHASE_SCENARIOS.values() for n in names}
    out["total"] = float(sum(out.values())) if out and set(metrics).issubset(known) else cost(metrics, scale)
    return out


def score(params: dict, scenarios=None, pool=None) -> tuple[float, dict]:
    """(cost, {scenario: metrics}). Scenarios run in parallel when a multiprocessing pool is given."""
    scenarios = scenarios_for(params) if scenarios is None else scenarios
    jobs = [(params, n) for n in scenarios]
    res = pool.starmap(run_scenario, jobs) if pool is not None else [run_scenario(*j) for j in jobs]
    metrics = dict(zip(scenarios, res))
    return cost(metrics, params["flying_height"]), metrics
