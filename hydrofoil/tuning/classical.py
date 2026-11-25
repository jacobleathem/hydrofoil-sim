"""
Classical starting points for the autotuner.

PID, from a relay feedback test (Astrom & Hagglund 1984): one loop at a time is switched to an ideal relay of
amplitude d about its trim while the other loops stay closed on their PIDs; the loop settles into a limit cycle of
amplitude a and period Pu, giving the ultimate gain Ku = 4 d / (pi a). The usual rule sets then give PID gains:

    Ziegler-Nichols classic (Ziegler & Nichols 1942)    Kp = 0.60 Ku   Ti = 0.50 Pu   Td = 0.125 Pu
    Z-N "some overshoot"                                 Kp = 0.33 Ku   Ti = 0.50 Pu   Td = 0.33 Pu
    Z-N "no overshoot"                                   Kp = 0.20 Ku   Ti = 0.50 Pu   Td = 0.33 Pu
    Tyreus-Luyben (Tyreus & Luyben 1992)                 Kp = 0.45 Ku   Ti = 2.2 Pu    Td = Pu / 6.3

(ki = Kp / Ti, kd = Kp Td; the roll loop stays PD as in the presets.) SIMC (Skogestad 2003) is not offered: it
needs a step-response model, and the heave loop is open-loop unstable, so no step test is possible.

LQR, from Bryson's rule (Bryson & Ho 1975: weight = 1 / acceptable excursion^2, which is how DEFAULT_WEIGHTS is
written) followed by an iterative re-weighting loop: fly the scenarios, compare each regulated quantity with its
target excursion, and scale its acceptable excursion by (target / achieved)^0.5 (tighter where it overshot, looser
where it has margin); the command-rate allowance is cut when the flaps hit their rate or torque limits. This loop is
a heuristic of this project, not a published method.
"""

from __future__ import annotations

import warnings

import numpy as np

RULES = {
    "Ziegler-Nichols": (0.60, 0.50, 0.125),
    "Z-N some overshoot": (0.33, 0.50, 0.33),
    "Z-N no overshoot": (0.20, 0.50, 0.33),
    "Tyreus-Luyben": (0.45, 2.2, 1 / 6.3),
}
LOOPS = {"heave": ("kp_heave", "ki_heave", "kd_heave"), "pitch": ("kp_pitch", "ki_pitch", "kd_pitch"),
         "roll": ("kp_roll", None, "kd_roll")}


def relay_test(params: dict, loop: str, d: float | None = None, T: float = 16.0) -> dict:
    """Relay experiment on one PID loop of the flying boat. Returns Ku, Pu and the measured amplitude."""
    warnings.simplefilter("ignore")
    from hydrofoil.vehicle import VehiclePhysics
    from hydrofoil.linearize import trim
    from hydrofoil.waves.airy import CalmWater
    p = dict(params, controller="pid", sensors="ideal", supervisor=False)
    vp = VehiclePhysics(p)
    x = None
    for v_try in (p["target_speed"], float((p.get("lqr_speeds") or (p["target_speed"],))[0])):
        try:
            x, _ = trim(vp, v_try, u0=(5.0, 0.0, 0.0, 200.0), max_iter=80)
            vp.params["target_speed"] = v_try
            break
        except RuntimeError:
            continue
    if x is None:
        return {"ok": False, "why": "cannot trim at target speed"}
    s = x.copy()
    if loop == "roll":
        s[3] = np.radians(1.0)                                 # kick the roll loop off zero so the relay starts
    d = d if d is not None else {"heave": 3.0, "pitch": 1.5, "roll": 1.5}[loop]
    pid = {"heave": vp.controller.heave_pid, "pitch": vp.controller.pitch_pid, "roll": vp.controller.roll_pid}[loop]
    hyst = {"heave": 0.002, "pitch": 0.05, "roll": 0.05}[loop]
    st = {"u": d}

    def relay(error, meas, integrate: bool = True):            # replaces this loop's PID output
        if error > hyst:
            st["u"] = d
        elif error < -hyst:
            st["u"] = -d
        return st["u"]
    pid.update = relay
    t, rec = 0.0, []
    for _ in range(int(T / p["dt"])):
        s, c, *_ = vp.step(s, t, CalmWater())
        t += p["dt"]
        if not np.all(np.isfinite(s)) or abs(np.degrees(s[3])) > 25:
            return {"ok": False, "why": "diverged during the relay test"}
        zref = vp.hull_offset - p["flying_height"]
        e = {"heave": s[2] - zref, "pitch": np.degrees(s[4]) - p["theta_ref"], "roll": np.degrees(s[3]) - p["phi_ref"]}[loop]
        rec.append((t, e))
    r = np.array(rec)
    late = r[r[:, 0] > T * 0.4]
    e = late[:, 1] - late[:, 1].mean()
    zc = late[1:, 0][(e[:-1] < 0) & (e[1:] >= 0)]               # upward zero crossings
    if len(zc) < 3:
        return {"ok": False, "why": "no sustained oscillation"}
    Pu = float(np.diff(zc).mean())
    a = float((e.max() - e.min()) / 2)
    Ku = 4 * d / (np.pi * a)
    return {"ok": True, "Ku": float(Ku), "Pu": Pu, "amplitude": a, "relay": d}


def pid_candidates(params: dict) -> dict:
    """{rule name: {gain key: value}} from relay tests on the heave, pitch and roll loops."""
    tests = {lp: relay_test(params, lp) for lp in LOOPS}
    out = {"_relay": tests}
    for name, (a, b, c) in RULES.items():
        g = {}
        for lp, (kp_k, ki_k, kd_k) in LOOPS.items():
            r = tests[lp]
            if not r["ok"]:
                g.update({kp_k: params[kp_k], kd_k: params[kd_k]})
                if ki_k:
                    g[ki_k] = params[ki_k]
                continue
            Kp, Ti, Td = a * r["Ku"], b * r["Pu"], c * r["Pu"]
            g[kp_k], g[kd_k] = Kp, Kp * Td
            if ki_k:
                g[ki_k] = Kp / Ti
        out[name] = g
    return out


LQR_TARGETS = {"z_rms": 0.03, "pitch_peak": 1.5, "roll_peak": 1.5, "flap_rate_rms": 60.0, "limited_frac": 0.02}


def bryson_update(mult: dict, metrics: dict, scale: float) -> dict:
    """One step of the iterative Bryson re-weighting (multipliers on the acceptable excursions)."""
    live = [(k, m) for k, m in metrics.items()
            if k not in ("takeoff", "supervised", "landing") and not m.get("failed")]
    if not live:
        live = [(k, m) for k, m in metrics.items() if not m.get("failed")]
    if not live:
        return {k: v * 0.8 for k, v in mult.items()}
    ok = [m for _, m in live]
    z = np.mean([m["z_rms"] for m in ok]) / (LQR_TARGETS["z_rms"] * scale)
    th = np.mean([m["pitch_peak"] for m in ok]) / LQR_TARGETS["pitch_peak"]
    ph = max((m.get("roll_rms_err", 0.0) + m.get("roll_overshoot", 0.0)) if k == "turn" else m["roll_peak"]
             for k, m in live) / LQR_TARGETS["roll_peak"]
    rate = np.mean([m["flap_rate_rms"] for m in ok]) / LQR_TARGETS["flap_rate_rms"]
    lim = max(m["limited_frac"] for m in ok) / LQR_TARGETS["limited_frac"]
    new = dict(mult)
    for k, ratio in (("z", z), ("iz", z), ("w", z), ("theta", th), ("q", th), ("phi", ph)):
        new[k] = float(np.clip(mult[k] * np.clip(ratio, 0.25, 4.0) ** -0.5, 0.1, 10.0))
    new["du"] = float(np.clip(mult["du"] * (0.8 if (rate > 1.0 or lim > 1.0) else 1.1), 0.1, 10.0))
    return new
