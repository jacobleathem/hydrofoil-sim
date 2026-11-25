"""
Main-foil flap and drive design study (docs/20): how flap geometry and gear ratio set the hinge torque,
the drive's speed and current, and the mechanical loads, on the manned preset by default.

    python tools/flap_design.py static                       # geometry table + gear-ratio window + mechanics
    python tools/flap_design.py fly                          # the preset as configured: one JSON line
    python tools/flap_design.py fly --cf 0.4 --sf 0.5 --sb 0 --slew 0 --rigid   # the pre-2026-11-16 manned flap

static: for each flap chord fraction c_f/c, flapped span fraction and hinge set-back, at takeoff / cruise / top
speed: lift effectiveness, trim flap, steady hinge moment at trim, hinge stiffness, the deflection and torque
needed for a fixed control authority (+-AUTH_DEG of effective angle of attack, the same lift change for every
geometry), and from that the gear-ratio window of the mj5208 + moteus r4.11 drive.

fly: re-derives the trim feed-forward for the geometry, designs the LQR on it and flies takeoff, head sea and
following sea with the servo drives in the loop; reports per-drive peaks and the actuator rail current.
"""

from __future__ import annotations

import argparse
import json
import sys
import warnings
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from hydrofoil.params import DEFAULT_PARAMS, MANNED_PARAMS            # noqa: E402
from hydrofoil.actuation.drive import DriveSpec                       # noqa: E402
from hydrofoil.actuation.hinge import thin_airfoil_coeffs             # noqa: E402

AUTH_DEG = 3.0          # control authority demanded of every geometry: +-3 deg of effective angle of attack
AUTH_RATE_DEG_S = 30.0  # and this rate of change of effective angle of attack (gust / wave following)
SF_TORQUE = 1.5         # design margin on torque and rate when picking the gear-ratio window

# Mechanics, PLACEHOLDER materials / sizes (docs/20 section 4)
E_STEEL, G_STEEL = 200e9, 79e9
TAU_ALLOW = 70e6        # Pa: 316 stainless shear yield 140 MPa / 2


def preset(name: str) -> dict:
    from hydrofoil.params import preset_params
    return preset_params(name)


def geometry_params(base: dict, cf: float | None = None, sf: float | None = None, sb: float | None = None,
                    N: float | None = None) -> dict:
    """The preset with another flap geometry (None = keep the preset's); the flap's structural inertia is rescaled
    with it (hinge.flap_inertia) so the drive sees the right load."""
    from hydrofoil.actuation.hinge import flap_inertia
    cf = base["main_flap_frac"] if cf is None else cf
    sf = base["main_flap_span_frac"] if sf is None else sf
    sb = base.get("main_flap_setback", 0.0) if sb is None else sb
    p = dict(base, main_flap_frac=cf, main_flap_span_frac=sf, main_flap_setback=sb)
    p["main_surface_inertia"] = flap_inertia(p["main_chord"], p["main_span"] * sf, cf, sb)
    if N is not None:
        p["gear_ratio"] = N
    return p


def rederive_trim_ff(p: dict) -> dict:
    """trim_ff_deg for this geometry: the level trim flap at trim_ff_speed (what the test in test_method_fixes
    checks for the defaults)."""
    from hydrofoil.vehicle import VehiclePhysics
    from hydrofoil.linearize import trim
    vp = VehiclePhysics(dict(p, controller="pid", sensors="ideal"))
    _x, u = trim(vp, p["trim_ff_speed"])
    return dict(p, trim_ff_deg=float(u[0]))


def static_point(p: dict, speed: float) -> dict:
    """Hinge quantities of one main-flap drive at level trim at `speed`."""
    from hydrofoil.vehicle import VehiclePhysics
    from hydrofoil.linearize import trim
    from hydrofoil.control.lqr import hinge_stiffness
    from hydrofoil.actuation.loads import steady_moments
    from hydrofoil.waves.airy import CalmWater
    warnings.simplefilter("ignore")
    vp = VehiclePhysics(dict(p, controller="pid", sensors="ideal"))
    eff = vp.main_flap_eff
    try:
        x, u = trim(vp, speed)
    except RuntimeError:
        return {"speed": speed, "eff": eff, "trim_deg": float("nan")}
    cmds = {"delta_main": u[0], "delta_elev": u[1], "delta_roll": 0.0}
    env = vp.wave_sample(x[:6], 0.0, CalmWater())
    m0 = steady_moments(vp, x, cmds, env, vp.hinges)["left"]
    k = hinge_stiffness(vp, "left", speed)
    d_auth = AUTH_DEG / eff                                   # flap deg for +-AUTH_DEG effective
    m_pk = abs(m0) + k * np.radians(d_auth)
    rate = AUTH_RATE_DEG_S / eff                              # flap deg/s
    return {"speed": speed, "eff": eff, "trim_deg": float(u[0]), "M_trim": float(m0), "k_hinge": float(k),
            "d_auth": d_auth, "d_max": abs(u[0]) + d_auth, "M_peak": float(m_pk), "rate_deg_s": rate,
            "P_out": float(m_pk * np.radians(rate))}


def gear_window(m_peak: float, rate_deg_s: float, m_hold: float, spec: DriveSpec | None = None) -> dict:
    """Gear ratios that give SF_TORQUE x the torque (motor peak / controller current) AND SF_TORQUE x the rate
    (back-EMF at the bus voltage), and the continuous current to hold m_hold."""
    s = spec or DriveSpec()
    m, c, g = s.motor, s.ctrl, s.gear
    tau_m = min(m.kt * c.i_peak, m.peak_torque)
    n_lo = SF_TORQUE * m_peak / (tau_m * g.efficiency)
    n_hi = c.k_mod * c.v_bus / m.kt / (SF_TORQUE * np.radians(rate_deg_s))
    return {"N_min_torque": n_lo, "N_max_speed": n_hi,
            "I_hold_at_20": m_hold / (20.0 * g.efficiency * m.kt), "kt": m.kt}


def mechanics(torque: float) -> dict:
    """Sizes the transmission must have for `torque` at the output (the drive can always push its full
    torque into a stop or a jam, so size for that, not the hydrodynamic moment)."""
    d_shaft = (16 * torque / (np.pi * TAU_ALLOW)) ** (1 / 3)
    out = {"torque": torque, "solid_shaft_mm": d_shaft * 1e3}
    for r in (0.02, 0.03, 0.04):                              # pushrod horn arm [m]
        out[f"pushrod_force_r{int(r*1e3)}"] = torque / r
    return out


def link_stiffness(L: float) -> list[dict]:
    """Rotational stiffness at the flap of candidate transmissions down a strut of length L."""
    rows = []
    for name, D, t in (("rod 8 mm", 0.008, None), ("tube 12x1", 0.012, 0.001), ("tube 16x1", 0.016, 0.001),
                       ("tube 20x1.5", 0.020, 0.0015)):
        d_in = 0.0 if t is None else D - 2 * t
        A = np.pi / 4 * (D**2 - d_in**2)
        I = np.pi / 64 * (D**4 - d_in**4)
        J = 2 * I
        r = 0.03
        rows.append({"name": name, "OD_mm": D * 1e3,
                     "push_k_rot_r30": E_STEEL * A / L * r * r,          # N m/rad at the flap, 30 mm horn
                     "push_buckle_N": np.pi**2 * E_STEEL * I / L**2,      # pinned-pinned, unguided
                     "torque_tube_k": G_STEEL * J / L})                   # N m/rad
    return rows


def cmd_static(a):
    base = preset(a.preset)
    speeds = {"manned": (6.0, 8.0, 9.5), "cad": (10.0, 12.0, 14.0)}.get(a.preset, (3.5, 4.0, 6.0))
    print(f"preset {a.preset}: chord {base['main_chord']} m, half-span {base['main_span']} m, "
          f"incidence {base.get('main_incidence_deg', 0)} deg, authority +-{AUTH_DEG} deg effective AoA "
          f"at {AUTH_RATE_DEG_S} deg/s")
    print("\nthin-airfoil coefficients (per rad; hinge moment ref q S_f c_f; viscous factor 0.8 applied in the sim)")
    print(f"{'cf':>5s}{'sb':>5s}{'tau':>7s}{'ch_a':>8s}{'ch_d':>8s}{'cm_d':>8s}{'M/auth':>8s}")
    for cf in (0.2, 0.25, 0.3, 0.4, 0.5):
        for sb in ((0.0, 0.1, 0.2, 0.3) if cf == 0.3 else (0.0,)):
            c = thin_airfoil_coeffs(cf, setback=sb)
            # hinge torque per unit lift change, relative to the 40 % plain flap: c_f^2 ch_d / tau
            ref = thin_airfoil_coeffs(0.4)
            rel = (cf**2 * c["ch_delta"] / c["tau"]) / (0.16 * ref["ch_delta"] / ref["tau"])
            print(f"{cf:5.2f}{sb:5.2f}{c['tau']:7.3f}{c['ch_alpha']:8.3f}{c['ch_delta']:8.3f}{c['cm_delta']:8.3f}{rel:8.2f}")

    print("\nper main-flap drive at level trim (M + = TE down; k = restoring hinge stiffness)")
    hdr = (f"{'cf':>5s}{'sf':>5s}{'sb':>5s}{'u':>5s}{'eff':>6s}{'trim':>7s}{'d_max':>7s}{'M_trim':>8s}"
           f"{'k':>7s}{'M_pk':>7s}{'rate':>7s}{'P_out':>7s}{'N_lo':>6s}{'N_hi':>6s}{'I_hold':>7s}")
    print(hdr)
    geoms = [(0.4, 0.5, 0.0), (0.3, 0.5, 0.0), (0.25, 0.5, 0.0), (0.2, 0.5, 0.0), (0.4, 1.0, 0.0),
             (0.25, 1.0, 0.0), (0.25, 0.75, 0.0), (0.4, 0.5, 0.2), (0.25, 0.5, 0.2), (0.25, 1.0, 0.15),
             (0.25, 1.0, 0.2)]
    worst = {}
    for cf, sf, sb in geoms:
        p = geometry_params(base, cf, sf, sb)
        for u in speeds:
            r = static_point(p, u)
            if not np.isfinite(r["trim_deg"]):
                print(f"{cf:5.2f}{sf:5.2f}{sb:5.2f}{u:5.1f}  no trim"); continue
            w = gear_window(r["M_peak"], r["rate_deg_s"], abs(r["M_trim"]))
            key = (cf, sf, sb)
            worst[key] = max(worst.get(key, 0.0), r["M_peak"])
            print(f"{cf:5.2f}{sf:5.2f}{sb:5.2f}{u:5.1f}{r['eff']:6.3f}{r['trim_deg']:7.1f}{r['d_max']:7.1f}"
                  f"{r['M_trim']:8.2f}{r['k_hinge']:7.1f}{r['M_peak']:7.2f}{r['rate_deg_s']:7.0f}{r['P_out']:7.1f}"
                  f"{w['N_min_torque']:6.1f}{w['N_max_speed']:6.1f}{w['I_hold_at_20']:7.2f}")

    print("\nmechanics: the drive can put its full output torque into a stop or a jam; size for that")
    s = DriveSpec()
    for N in (10, 14, 20, 30):
        t_full = min(s.motor.kt * s.ctrl.i_peak, s.motor.peak_torque) * N * s.gear.efficiency
        mm = mechanics(t_full)
        print(f"  N {N:3d}: full torque {t_full:5.1f} N m -> solid shaft >= {mm['solid_shaft_mm']:4.1f} mm (SF 2), "
              f"pushrod force {mm['pushrod_force_r20']:5.0f} / {mm['pushrod_force_r30']:5.0f} / "
              f"{mm['pushrod_force_r40']:5.0f} N at 20 / 30 / 40 mm horn")
    L = base["main_pos_z"] - 0.1
    print(f"\ntransmission down a {L:.2f} m strut (steel), stiffness at the flap vs servo kp "
          f"{250:.0f} N m/rad (series: k_eff = 1/(1/kp + 1/k_link))")
    for r in link_stiffness(L):
        print(f"  {r['name']:12s} pushrod (30 mm horn) {r['push_k_rot_r30']:8.0f} N m/rad, buckles unguided at "
              f"{r['push_buckle_N']:6.0f} N | torque tube {r['torque_tube_k']:6.0f} N m/rad")


def fly(p: dict, scenarios=("takeoff", "head", "following", "head_top")) -> dict:
    """Closed loop on the preset's controller with the servo drives. head_top = head sea at the top LQR speed."""
    from hydrofoil.vehicle import VehiclePhysics
    from hydrofoil.waves.airy import AiryWave, CalmWater
    from hydrofoil.linearize import trim
    warnings.simplefilter("ignore")
    p = dict(p, controller="lqr", actuator_model="servo")
    out = {"eff": None, "trim_ff_deg": p["trim_ff_deg"]}
    dt, vbus = p["dt"], p["v_bus"]
    zr = p["hull_offset"] - p["flying_height"]
    amp, lam = 0.3 * p["flying_height"], 3.0 * p["hull_length"]
    v_top = max(p.get("lqr_speeds", (p["target_speed"],)))
    for name in scenarios:
        q = dict(p, target_speed=v_top) if name == "head_top" else dict(p)
        vp = VehiclePhysics(q)
        out["eff"] = vp.main_flap_eff
        if name == "takeoff":
            s, T, wave, t0 = vp.rest_state(), 40.0, CalmWater(), 0.0
        else:
            s, _ = trim(vp, q["target_speed"]); T, t0 = 20.0, 3.0
            wave = AiryWave(amp, lam, direction=0.0 if name == "following" else np.pi)
        t = 0.0
        keys = ("M", "tau", "I", "V", "lim", "ang", "twist", "gap")
        rec = {d: {k: [] for k in keys} for d in ("left", "right", "elev")}
        rail, z, fail = [], [], False
        while t < T:
            s, c, _th, draft, _ = vp.step(s, t, wave); t += dt
            if not np.all(np.isfinite(s)) or abs(s[3]) > np.radians(30):
                fail = True; break
            if t < t0:
                continue
            dr = c["drive"]
            for d in rec:
                R = rec[d]
                R["M"].append(dr[d]["M"]); R["tau"].append(dr[d]["tau_out"]); R["I"].append(dr[d]["I"])
                R["V"].append(dr[d]["V"]); R["lim"].append(dr[d]["limited"])
                R["twist"].append(dr[d].get("twist_deg", 0.0)); R["gap"].append(dr[d].get("gap_deg", 0.0))
            rec["left"]["ang"].append(c["delta_main"] - c["delta_roll"]); rec["elev"]["ang"].append(c["delta_elev"])
            rail.append(sum(dr[d]["P"] for d in rec) / vbus)
            z.append(s[2] - zr)
        r = {"failed": fail, "speed": q["target_speed"]}
        for d in ("left", "elev"):
            R = rec[d]
            if not R["M"]:
                continue
            I = np.array(R["I"])
            r[d] = {"M_pk": float(np.max(np.abs(R["M"]))), "M_rms": float(np.sqrt(np.mean(np.square(R["M"])))),
                    "tau_pk": float(np.max(np.abs(R["tau"]))), "I_pk": float(np.max(np.abs(I))),
                    "I_rms": float(np.sqrt(np.mean(I**2))), "V_pk": float(np.max(R["V"])),
                    "lim_frac": float(np.mean(R["lim"])),
                    "ang_min": float(np.min(R["ang"])), "ang_max": float(np.max(R["ang"])),
                    "rate_pk": float(np.max(np.abs(np.diff(R["ang"]))) / dt),
                    "twist_pk": float(np.max(np.abs(R["twist"]))), "gap_pk": float(np.max(np.abs(R["gap"]))),
                    "gear_torque_pk": float(vp.drives[d]._peak_spring)}
        if rail:
            r["rail_pk"], r["rail_mean"] = float(np.max(rail)), float(np.mean(rail))
            zz = np.array(z[int(2.0 / dt):]) if name == "takeoff" else np.array(z)
            r["z_rms_mm"] = float(np.sqrt(np.mean(zz**2)) * 1e3) if len(zz) else float("nan")
            if name == "takeoff":                          # height after it has settled (last 10 s)
                r["z_end_rms_mm"] = float(np.sqrt(np.mean(np.array(z[-int(10.0 / dt):]) ** 2)) * 1e3)
        out[name] = r
    return out


def cmd_fly(a):
    p = geometry_params(preset(a.preset), a.cf, a.sf, a.sb, a.N)
    if a.rigid:                                   # the drive train as it was before backlash / twist were modelled
        p["servo_params"] = dict(p.get("servo_params") or {}, backlash_deg=0.0, gear_stiffness=None, encoder="motor")
        p["main_flap_GJ"] = p["elev_GJ"] = None
    if a.encoder:
        p["servo_params"] = dict(p.get("servo_params") or {}, encoder=a.encoder)
    if a.backlash is not None:
        p["servo_params"] = dict(p.get("servo_params") or {}, backlash_deg=a.backlash)
    if a.slew is not None:
        p["cmd_slew_deg_s"] = a.slew or None
    p = rederive_trim_ff(p)
    r = fly(p)
    print(json.dumps({"preset": a.preset, "cf": p["main_flap_frac"], "sf": p["main_flap_span_frac"],
                      "sb": p.get("main_flap_setback", 0.0), "N": p["gear_ratio"], "rigid": a.rigid,
                      "encoder": (p.get("servo_params") or {}).get("encoder", "motor"),
                      "backlash": (p.get("servo_params") or {}).get("backlash_deg", 0.0),
                      "slew": p.get("cmd_slew_deg_s"), "J_flap": p["main_surface_inertia"], **r}), flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest="cmd", required=True)
    s = sp.add_parser("static"); s.add_argument("--preset", default="manned", choices=("manned", "model", "cad"))
    f = sp.add_parser("fly")
    f.add_argument("--preset", default="manned", choices=("manned", "model", "cad"))
    f.add_argument("--cf", type=float, help="flap chord / foil chord (default: the preset's)")
    f.add_argument("--sf", type=float, help="flapped fraction of each half-span (default: the preset's)")
    f.add_argument("--sb", type=float, help="hinge set-back, fraction of flap chord (default: the preset's)")
    f.add_argument("--N", type=float, help="gear ratio (default: the preset's)")
    f.add_argument("--rigid", action="store_true", help="no backlash, rigid gear, no twist, motor encoder")
    f.add_argument("--encoder", choices=("motor", "output"), help="override the encoder position")
    f.add_argument("--backlash", type=float, help="override backlash [deg]")
    f.add_argument("--slew", type=float, help="command slew limit [deg/s], 0 = off (default: the preset's)")
    a = ap.parse_args()
    {"static": cmd_static, "fly": cmd_fly}[a.cmd](a)


if __name__ == "__main__":
    main()
