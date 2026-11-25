"""
Co-dependency checker: parameters that are DERIVED from, or LIMITED by, other parameters, checked against each
other so a change to one cannot silently invalidate another (docs/20 section 8).

    python -m hydrofoil.consistency                 # both presets
    python -m hydrofoil.consistency --preset manned

Each check returns a row (name, value, limit, status, note). Status:
    FAIL  the parameters contradict each other or the model (a derived value is stale, a part is overstressed,
          a configured limit cannot be reached or is exceeded by the drive's own capability)
    WARN  a sizing margin is thin or an assumption is outside its range; flying is possible but the number
          needs a decision (often a firmware limit or a hardware choice)
    OK
tests/test_consistency.py requires no FAIL on either preset.

Firmware note: the real drives are expected to run a firmware "co-dependency limit" (coupled current / torque /
rail / temperature limits across the drives, keeping everything inside a safe operating area). The checks below
size against the configured `torque_cap`, `drive_i_cont` and `supply_i_max`; they stand in for that limit until
its rules exist.
"""

from __future__ import annotations

import argparse
import warnings

import numpy as np

SF_TORQUE = 1.5          # margin of the firmware torque cap over the hinge demand at trim plus authority
AUTH_DEG = 3.0           # +-effective angle-of-attack authority assumed for the static sizing checks
PLAIN_FLAP_LINEAR_DEG = 20.0   # plain flaps lose effectiveness past about 15-20 deg (not modelled)
TAU_ALLOW = 70e6         # Pa, 316 stainless shear yield 140 MPa / 2 (shaft static check)


def _row(name, value, limit, ok, warn_only=False, note=""):
    status = "OK" if ok else ("WARN" if warn_only else "FAIL")
    return {"check": name, "value": value, "limit": limit, "status": status, "note": note}


def check(params: dict) -> list[dict]:
    warnings.simplefilter("ignore")
    from hydrofoil.vehicle import VehiclePhysics
    from hydrofoil.linearize import trim
    from hydrofoil.control.lqr import hinge_stiffness
    from hydrofoil.actuation.loads import steady_moments
    from hydrofoil.actuation.hinge import thin_airfoil_coeffs
    from hydrofoil.actuation.servo import DEFAULT_SERVO_PARAMS
    from hydrofoil.forces.foil import main_flap_effectiveness
    from hydrofoil.waves.airy import CalmWater

    p = dict(params)
    vp = VehiclePhysics(dict(p, controller="pid", sensors="ideal", supervisor=False))
    rows = []
    sp = dict(DEFAULT_SERVO_PARAMS, **(p.get("servo_params") or {}))
    d = vp.drives["left"]
    spec = d.spec
    m, c, g = spec.motor, spec.ctrl, spec.gear
    N, eta, kt = g.ratio, g.efficiency, m.kt
    full = min(kt * c.i_peak, m.peak_torque) * N * eta                 # what the drive can push at the output
    cap = sp["torque_cap"]
    flap = p.get("main_actuation", "flap") == "flap"

    # 1. derived values that must match what they are derived from
    if flap and p.get("main_flap_effectiveness") is None:
        e = main_flap_effectiveness(p["main_flap_frac"], p["main_flap_span_frac"])
        rows.append(_row("flap effectiveness = f(geometry)", vp.main_flap_eff, e, abs(vp.main_flap_eff - e) < 1e-12))
    if p.get("trim_ff_deg"):
        try:
            _x, u = trim(vp, p["trim_ff_speed"])
            rows.append(_row("trim_ff_deg = trim flap at trim_ff_speed", p["trim_ff_deg"], round(float(u[0]), 2),
                             abs(u[0] - p["trim_ff_deg"]) < 0.3, note="re-derive with linearize.trim after any geometry/mass edit"))
        except RuntimeError as ex:
            rows.append(_row("trim_ff_deg = trim flap at trim_ff_speed", p["trim_ff_deg"], "no trim", False, note=str(ex)))
    tuned = p.get("tuned_for")
    if tuned:
        diff = {k: (v, p.get(k)) for k, v in tuned.items() if p.get(k) != v}
        rows.append(_row("gains / LQR weights tuned on this geometry", "same" if not diff else diff, "tuned_for",
                         not diff, warn_only=True, note="re-run hydrofoil.tuning.autotune and update tuned_for"))

    # 2. torque path: firmware cap vs what the hardware can give and take
    rows.append(_row("torque_cap <= drive full torque (else the cap is never reached)", cap, round(full, 2),
                     cap <= full + 1e-9, warn_only=True))
    rows.append(_row("torque_cap <= gearbox rated torque", cap, g.rated_torque, cap <= g.rated_torque))
    rows.append(_row("drive full torque <= gearbox rated (stall into a stop or jam)", round(full, 2), g.rated_torque,
                     full <= g.rated_torque, warn_only=True, note="else the firmware cap is the only protection; add a slip clutch"))
    i_cap = cap / (N * eta * kt)
    rows.append(_row("current at torque_cap <= controller peak", round(i_cap, 1), c.i_peak, i_cap <= c.i_peak))
    t_shaft = min(cap, full)
    shaft_d = max(float(p.get("shaft_diameter") or spec.shaft_diameter), 1e-4)
    stress = 16 * t_shaft / (np.pi * shaft_d ** 3)
    rows.append(_row("output shaft shear at min(cap, full) <= allowable", round(stress / 1e6, 1), TAU_ALLOW / 1e6,
                     stress <= TAU_ALLOW, note=f"needs >= {1e3 * (16 * t_shaft / (np.pi * TAU_ALLOW)) ** (1 / 3):.1f} mm"))

    # 3. trim across the flight envelope: exists, inside the flap's useful range, holdable
    speeds = sorted(set(list(p.get("lqr_speeds", (3.0, 4.0, 5.0, 6.0, 7.0))) + [p["target_speed"]]))
    worst_m, worst_hold, worst_d = 0.0, 0.0, 0.0
    for u in speeds:
        try:
            x, uu = trim(vp, u)
        except RuntimeError:
            rows.append(_row(f"trim exists at {u} m/s", "none", "-", False, note="LQR schedule / target speed outside the trimmable range"))
            continue
        cmds = {"delta_main": uu[0], "delta_elev": uu[1], "delta_roll": 0.0}
        env = vp.wave_sample(x[:6], 0.0, CalmWater())
        m0 = abs(steady_moments(vp, x, cmds, env, vp.hinges)["left"])
        k = hinge_stiffness(vp, "left", u) if flap else 0.0
        d_auth = AUTH_DEG / max(vp.main_flap_eff, 1e-4)
        worst_m = max(worst_m, m0 + k * np.radians(d_auth))
        worst_hold = max(worst_hold, m0)
        worst_d = max(worst_d, abs(uu[0]) + d_auth)
        rows.append(_row(f"trim main flap at {u} m/s inside +-{PLAIN_FLAP_LINEAR_DEG:.0f} deg less authority",
                         round(float(uu[0]), 2), round(PLAIN_FLAP_LINEAR_DEG - d_auth, 2),
                         abs(uu[0]) + d_auth <= PLAIN_FLAP_LINEAR_DEG, warn_only=True))
    rows.append(_row(f"hinge demand (trim + {AUTH_DEG:.0f} deg authority) x {SF_TORQUE} <= torque_cap",
                     round(worst_m * SF_TORQUE, 2), cap, worst_m * SF_TORQUE <= cap, warn_only=True,
                     note="static sizing assumption; the closed loop is the real test (tools/flap_design.py fly)"))
    i_hold = worst_hold / (N * eta * kt)
    rows.append(_row("holding current at worst trim <= drive_i_cont (thermal)", round(i_hold, 1), c.i_cont,
                     i_hold <= c.i_cont, warn_only=True,
                     note="continuous rating depends on cooling; firmware co-dependency limit decides"))
    p_hold = 3 * (0.75 * m.r_line_line * (worst_hold / (N * eta * kt)) ** 2 + c.p_idle)
    rows.append(_row("three drives holding trim: rail current <= supply_i_max", round(p_hold / c.v_bus, 2),
                     c.supply_i_max, p_hold / c.v_bus <= c.supply_i_max, warn_only=True))

    # 4. servo loop vs the load and the drive train
    for name in ("left", "elev"):
        kh = hinge_stiffness(vp, name, p["target_speed"])
        rows.append(_row(f"{name}: hinge stiffness < servo kp at cruise (else the surface stays well short of its command)",
                         round(kh, 1), sp["kp"], kh < sp["kp"], warn_only=True))
    kg = sp.get("gear_stiffness")
    if kg is not None:
        rows.append(_row("gear stiffness >= 5 x servo kp", kg, 5 * sp["kp"], kg >= 5 * sp["kp"], warn_only=True,
                         note="drive-train compliance in series with the position loop"))
    compliant = sp.get("backlash_deg", 0.0) > 0 or kg is not None
    if compliant and sp.get("encoder", "motor") == "output":
        rows.append(_row("compliant drive train with output-only feedback", "output", "dual", False, warn_only=True,
                         note="non-collocated damping rings (72 N m through the gear at takeoff); use 'dual'"))
    if sp.get("backlash_deg", 0.0) > 0 and sp.get("encoder", "motor") == "motor":
        rows.append(_row("backlash inside the position loop", sp["backlash_deg"], "output encoder", False, warn_only=True,
                         note="motor-side encoder: free play is outside the loop (limit-cycle risk with integral action)"))
    tk = vp.twist_stiffness("left")
    if tk is not None and worst_hold > 0:
        tw = np.degrees(worst_m / tk)
        rows.append(_row("main flap twist at the worst hinge demand <= 0.5 deg", round(tw, 3), 0.5, tw <= 0.5, warn_only=True))

    # 5. hinge balance: set-back must not overbalance; structural inertia must follow the flap geometry
    if flap:
        from hydrofoil.actuation.hinge import flap_inertia
        c_flap = p.get("main_flap_chord")
        c_main_hinge = float(c_flap) / max(float(p["main_flap_frac"]), 1e-4) if (c_flap is not None and float(c_flap) > 0.0) else p["main_chord"]
        j = flap_inertia(c_main_hinge, p["main_span"] * p["main_flap_span_frac"], p["main_flap_frac"],
                         p.get("main_flap_setback", 0.0))
        rows.append(_row("main_surface_inertia matches the flap geometry (+-30 %)", p["main_surface_inertia"],
                         float(f"{j:.3g}"), abs(p["main_surface_inertia"] / max(j, 1e-9) - 1.0) <= 0.3, warn_only=True,
                         note="hinge.flap_inertia scaling; replace both with the CAD value"))
        co = thin_airfoil_coeffs(p["main_flap_frac"], setback=p.get("main_flap_setback", 0.0))
        rows.append(_row("main flap not overbalanced (Ch_delta < 0, Ch_alpha < 0)", round(co["ch_delta"], 3), "< 0",
                         co["ch_delta"] < 0 and co["ch_alpha"] < 0))
        sb = p.get("main_flap_setback", 0.0)
        if sb > 0:
            lo, hi = sb, 0.6
            for _ in range(40):                                      # set-back where Ch_delta crosses zero
                mid = 0.5 * (lo + hi)
                lo, hi = (mid, hi) if thin_airfoil_coeffs(p["main_flap_frac"], setback=mid)["ch_delta"] < 0 else (lo, mid)
            rows.append(_row("set-back <= 0.6 x thin-theory overbalance point", sb, round(0.6 * lo, 3), sb <= 0.6 * lo,
                             warn_only=True, note="thin theory overstates balance; keep margin"))
    if p.get("elev_actuation") == "all_moving":
        x_ac = thin_airfoil_coeffs(0.0)["x_ac"]
        rows.append(_row("elevator pivot ahead of its aerodynamic centre (weathervane-stable)", p["elev_pivot_frac"],
                         round(x_ac, 3), p["elev_pivot_frac"] < x_ac))

    # 6. commands, allocator, and split-flap parameters vs stops
    cmd_max = float(p.get("flap_cmd_max_deg", 20.0))
    rows.append(_row("flap command limit <= flap stop", cmd_max, p["flap_stop_deg"], cmd_max <= p["flap_stop_deg"]))
    from hydrofoil.control.allocator import SAFETY_PRESETS
    sm = str(p.get("safety_margin", "standard")).lower()
    rows.append(_row("safety_margin preset valid", sm, tuple(SAFETY_PRESETS), sm in SAFETY_PRESETS))
    al = str(p.get("allocator", "jacobian")).lower()
    rows.append(_row("allocator mode valid", al, ("jacobian", "legacy"), al in ("jacobian", "legacy")))
    k_cc = float(p.get("split_flap_cross_coupling", 0.08))
    rows.append(_row("split_flap_cross_coupling in [0, 0.35]", k_cc, "0..0.35", 0.0 <= k_cc <= 0.35))
    if p.get("cmd_slew_deg_s"):
        v_rate = np.degrees(c.k_mod * c.v_bus / kt / N)
        rows.append(_row("command slew <= drive no-load rate", p["cmd_slew_deg_s"], round(v_rate, 0),
                         p["cmd_slew_deg_s"] <= v_rate, warn_only=True))
    return rows


def report(params: dict, title: str = "") -> list[dict]:
    rows = check(params)
    if title:
        print(f"== {title}")
    for r in rows:
        print(f"  {r['status']:4s}  {r['check']}: {r['value']} (limit {r['limit']})" + (f"  [{r['note']}]" if r["note"] and r["status"] != "OK" else ""))
    return rows


def main():
    from hydrofoil.params import PRESETS
    ap = argparse.ArgumentParser(description="Check parameter co-dependencies")
    ap.add_argument("--preset", choices=(*PRESETS, "both", "all"), default="both",
                    help="both = model + manned (the tuned presets); all = also the CAD boat")
    a = ap.parse_args()
    fails = 0
    for name, p in PRESETS.items():
        if a.preset == name or a.preset == "all" or (a.preset == "both" and name in ("model", "manned")):
            fails += sum(r["status"] == "FAIL" for r in report(p, name))
    raise SystemExit(1 if fails else 0)


if __name__ == "__main__":
    main()
