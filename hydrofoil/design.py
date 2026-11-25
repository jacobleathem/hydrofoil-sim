"""
Boat design layer for the editor and the autotuner: turns design edits into a consistent parameter set, and works
out what they do (takeoff speed, cruise trim, power, margins) without flying the boat.

Edits (`overrides`, all optional; anything else that is a params key is copied through):
    payloads        list of {"name", "mass" [kg], "x" [m fwd of the nominal CG], "y" [m to starboard], "z" [m up]}.
                    They replace the preset's own `default_payloads` (the manned preset carries a 75 kg rider);
                    the empty boat is mass - sum(default payloads). Total mass, CG offset (x, y) and inertia follow.
                    The CG height is not modelled (the sim has no vertical CG offset) and is reported only.
    strut_length    hull bottom to main foil [m]; moves both foils (elevator stays 0.1 m higher)
    main_span       per half [m]; the halves stay 0.1 m apart at the root
    main_chord, main_incidence_deg, main_flap_frac, elev_chord, elev_span, elev_pos_x, flying_height, target_speed ...

When anything that changes trim is edited, the trim feed-forward and the LQR / estimator speed schedule are
re-derived from the new boat (the speeds where it can trim with the flap inside +-15 deg).

Foil "shape" is limited by the model: rectangular strip foils with one section polar (NACA 0009), so span, chord,
aspect ratio, flap fraction and incidence are what can change; taper, sweep, twist and section are not modelled.
"""

from __future__ import annotations

import warnings

import numpy as np

from hydrofoil.constants import RHO_WATER, G
from hydrofoil.actuation.hinge import flap_inertia
from hydrofoil.actuation.drive import DriveSpec
from hydrofoil.actuation.servo import DEFAULT_SERVO_PARAMS
from hydrofoil.dynamics.rigid_body import strip_added_mass_heave_pitch, strip_added_mass_roll
from hydrofoil.forces.foil import main_flap_effectiveness, MAIN_FOIL_FLAP_EFFECTIVENESS

TRIM_EDIT_KEYS = {"payloads", "mass", "empty_mass", "strut_length", "main_span", "main_chord", "main_incidence_deg", "main_flap_frac",
                  "main_flap_span_frac", "main_flap_setback", "main_flap_effectiveness",
                  "elev_chord", "elev_span", "elev_pos_x", "main_pos_x", "flying_height", "hull_offset", "cg_offset", "cg_z",
                  "hull_length", "hull_beam"}
FLAP_TRIM_LIMIT = 15.0                                    # deg of +-20: margin kept for waves and control


def _point_inertia(m, r):
    r = np.asarray(r, float)
    return m * (np.dot(r, r) * np.eye(3) - np.outer(r, r))


def _sanitize_payloads(loads_raw) -> list[dict]:
    """Normalize a list of payload dicts so missing or None fields default cleanly to zero."""
    return [
        {"name": str((q or {}).get("name") or "Load"),
         "mass": max(float((q or {}).get("mass") or 0.0), 0.0),
         "x": float((q or {}).get("x") or 0.0),
         "y": float((q or {}).get("y") or 0.0),
         "z": float((q or {}).get("z") or 0.0)}
        for q in (loads_raw or ())
    ]


def mass_properties(p: dict, payloads: list | None, empty_mass: float | None = None,
                    hull_scale: tuple[float, float] = (1.0, 1.0)) -> dict:
    """Total mass, CG (relative to the nominal CG) and inertia about the new CG for the given payload list."""
    defaults = _sanitize_payloads(p.get("_base_payloads", p.get("default_payloads", [])))
    base_mass = float(p.get("_base_mass", p["mass"]))
    base_I = p.get("_base_inertia", (p["ixx"], p["iyy"], p["izz"]))
    m_def = sum(q["mass"] for q in defaults)
    m_e0 = base_mass - m_def
    if m_e0 <= 0.0:
        raise ValueError(f"total mass ({base_mass} kg) must exceed default payload mass ({m_def} kg)")
    I_nom = np.diag([float(x) for x in base_I])                       # boat + default payloads, about the nominal CG
    I_empty = I_nom - sum((_point_inertia(q["mass"], (q["x"], q["y"], q["z"])) for q in defaults), np.zeros((3, 3)))
    # the empty boat's own CG: nominal CG = (m_e0 r_e + sum m_i r_i) / base_mass = 0
    r_e = -sum((q["mass"] * np.array([q["x"], q["y"], q["z"]]) for q in defaults), np.zeros(3)) / m_e0
    if empty_mass is not None:
        m_empty = float(empty_mass)
        if m_empty <= 0.0:
            raise ValueError(f"empty_mass must be > 0, got {m_empty}")
    else:
        cur_def = _sanitize_payloads(p.get("default_payloads", defaults))
        m_cur_empty = float(p.get("mass_empty", float(p["mass"]) - sum(q["mass"] for q in cur_def)))
        m_empty = m_cur_empty if m_cur_empty > 0.0 else m_e0
    sm = m_empty / m_e0
    sb, sl = float(hull_scale[0]), float(hull_scale[1])
    I_e_cm = I_empty - _point_inertia(m_e0, r_e)                      # empty boat about its own CG
    scale_vec = np.array([sm * sb * sb, sm * sl * sl, sm * 0.5 * (sb * sb + sl * sl)])
    I_e_cm = np.diag(np.maximum(np.diag(I_e_cm) * scale_vec, 1e-4))
    loads = _sanitize_payloads(p.get("default_payloads", defaults) if payloads is None else payloads)
    m = m_empty + sum(q["mass"] for q in loads)
    if m <= 0.0:
        raise ValueError(f"resulting total mass must be > 0, got {m}")
    cg = (m_empty * r_e + sum((q["mass"] * np.array([q["x"], q["y"], q["z"]]) for q in loads), np.zeros(3))) / m
    # I_e_cm is about the empty boat's CG; move everything to the combined CG via parallel axis
    I = I_e_cm + _point_inertia(m_empty, r_e - cg)
    for q in loads:
        I = I + _point_inertia(q["mass"], np.array([q["x"], q["y"], q["z"]]) - cg)
    return {"mass": float(m), "mass_empty": float(m_empty), "cg": cg.tolist(), "inertia": np.diag(I).tolist(),
            "payloads": loads}


def _wetted_proxy(p: dict) -> float:
    L, B = p["hull_length"], p["hull_beam"]
    n = 2 if float(p.get("hull_twin_spacing", 0.0) or 0.0) > 0.0 else 1
    d = min(p["mass"] / max(RHO_WATER * n * L * B, 1e-6), p["max_draft"])
    return n * (L * B + 2.0 * d * (L + B))


def _foil_eff(p: dict) -> float:
    if p.get("main_flap_effectiveness") is not None:
        return float(p["main_flap_effectiveness"])
    if p.get("main_actuation", "flap") == "flap":
        return float(main_flap_effectiveness(p["main_flap_frac"], p["main_flap_span_frac"]))
    return MAIN_FOIL_FLAP_EFFECTIVENESS


def _scale_inertias_and_hull(old: dict, p: dict, ov: dict):
    """Scale empty-boat dry inertias, hull drag, and critical rotational damping on boat-size edits."""
    defaults = _sanitize_payloads(p.get("default_payloads", []))
    m_def = sum(q["mass"] for q in defaults)
    m_e_old = old["mass"] - m_def
    m_e_new = p["mass"] - m_def
    sb = p["hull_beam"] / max(old["hull_beam"], 1e-4)
    sl = p["hull_length"] / max(old["hull_length"], 1e-4)
    if ("mass" in ov or "hull_length" in ov or "hull_beam" in ov) and not any(
        k in ov for k in ("ixx", "iyy", "izz", "payloads", "empty_mass")
    ):
        if m_e_old > 0.0 and m_e_new > 0.0:
            sm = m_e_new / m_e_old
            I_nom = np.diag([old["ixx"], old["iyy"], old["izz"]])
            I_empty = I_nom - sum((_point_inertia(q["mass"], (q["x"], q["y"], q["z"])) for q in defaults), np.zeros((3, 3)))
            r_e = -sum((q["mass"] * np.array([q["x"], q["y"], q["z"]]) for q in defaults), np.zeros(3)) / m_e_old
            I_e_cm = I_empty - _point_inertia(m_e_old, r_e)
            scale_vec = np.array([sm * sb * sb, sm * sl * sl, sm * 0.5 * (sb * sb + sl * sl)])
            I_e_scaled = np.diag(np.maximum(np.diag(I_e_cm) * scale_vec, 1e-4))
            cg = (m_e_new * r_e + sum((q["mass"] * np.array([q["x"], q["y"], q["z"]]) for q in defaults), np.zeros(3))) / p["mass"]
            I = I_e_scaled + _point_inertia(m_e_new, r_e - cg)
            for q in defaults:
                I = I + _point_inertia(q["mass"], np.array([q["x"], q["y"], q["z"]]) - cg)
        else:
            # Whole-craft mass override below default payload mass: scale total dry inertias directly
            sm = max(p["mass"] / max(old["mass"], 1e-6), 1e-4)
            I = np.diag([old["ixx"] * sm * sb * sb, old["iyy"] * sm * sl * sl,
                         old["izz"] * sm * 0.5 * (sb * sb + sl * sl)])
        if "ixx" not in ov:
            p["ixx"] = round(float(I[0, 0]), 3)
        if "iyy" not in ov:
            p["iyy"] = round(float(I[1, 1]), 3)
        if "izz" not in ov:
            p["izz"] = round(float(I[2, 2]), 3)
    if ({"hull_length", "hull_beam", "max_draft", "mass", "payloads"} & set(ov)) and "hull_drag_coeff" not in ov:
        p["hull_drag_coeff"] = round(old["hull_drag_coeff"] * (_wetted_proxy(p) / max(_wetted_proxy(old), 1e-6)), 2)
    if ({"hull_length", "hull_beam", "max_draft", "mass", "payloads", "iyy"} & set(ov)) and "hull_rot_damping" not in ov:
        k_old = old["hull_beam"] * old["hull_length"] ** 3 * old["iyy"]
        k_new = p["hull_beam"] * p["hull_length"] ** 3 * p["iyy"]
        p["hull_rot_damping"] = round(old["hull_rot_damping"] * float(np.sqrt(k_new / max(k_old, 1e-9))), 2)


def _scale_structures_drives_and_gains(old: dict, p: dict, ov: dict):
    """Auto-derive surface inertia, GJ scaling, shaft sizing, propulsion scaling, and first-order PID scaling."""
    main_keys = {"main_chord", "main_span", "main_flap_frac", "main_flap_span_frac", "main_flap_setback"}
    if main_keys & set(ov):
        if "main_surface_inertia" not in ov and p.get("main_actuation", "flap") == "flap":
            j = flap_inertia(p["main_chord"], p["main_span"] * p.get("main_flap_span_frac", 1.0),
                             p["main_flap_frac"], p.get("main_flap_setback", 0.0))
            p["main_surface_inertia"] = float(f"{j:.3g}")
        if "main_flap_GJ" not in ov and p.get("main_flap_GJ") is not None:
            cf_o = max(old["main_chord"] * old["main_flap_frac"], 1e-4)
            cf_n = max(p["main_chord"] * p["main_flap_frac"], 1e-4)
            p["main_flap_GJ"] = round(old["main_flap_GJ"] * (p["main_chord"] / max(old["main_chord"], 1e-4)) * (cf_n / cf_o) ** 3, 2)
    if "elev_chord" in ov and "elev_GJ" not in ov and p.get("elev_GJ") is not None:
        p["elev_GJ"] = round(old["elev_GJ"] * (p["elev_chord"] / max(old["elev_chord"], 1e-4)) ** 4, 2)
    if ({"servo_params", "gear_ratio"} & set(ov)) and "shaft_diameter" not in ov:
        spec = DriveSpec()
        N = p["gear_ratio"]
        sp = dict(DEFAULT_SERVO_PARAMS, **(p.get("servo_params") or {}))
        full = min(spec.motor.kt * spec.ctrl.i_peak, spec.motor.peak_torque) * N * spec.gear.efficiency
        t_shaft = min(sp["torque_cap"], full)
        d_req = float((16.0 * t_shaft * 1.5 / (np.pi * 70e6)) ** (1.0 / 3.0))
        p["shaft_diameter"] = max(float(old.get("shaft_diameter", spec.shaft_diameter)), round(d_req, 4))
    if {"mass", "payloads", "target_speed"} & set(ov):
        m_old = max(old["mass"], 1e-4)
        if "max_thrust" not in ov and ({"mass", "payloads"} & set(ov)):
            p["max_thrust"] = round(old["max_thrust"] * (p["mass"] / m_old), 1)
        if "prop_power_max" not in ov and p.get("prop_power_max") is not None:
            p["prop_power_max"] = round(old["prop_power_max"] * (p["mass"] / m_old)
                                         * (p["target_speed"] / max(old["target_speed"], 0.5)), 1)
    # First-order control-authority gain scaling so resized craft stay flyable before re-autotuning
    if (main_keys | {"mass", "payloads", "elev_chord", "elev_span", "elev_pos_x", "main_lateral_offset", "ixx", "iyy"}) & set(ov):
        mh_o, ip_o = strip_added_mass_heave_pitch([(old["main_chord"], old["main_span"], old["main_pos_x"]),
                                                   (old["main_chord"], old["main_span"], old["main_pos_x"]),
                                                   (old["elev_chord"], old["elev_span"], old["elev_pos_x"])])
        mh_n, ip_n = strip_added_mass_heave_pitch([(p["main_chord"], p["main_span"], p["main_pos_x"]),
                                                   (p["main_chord"], p["main_span"], p["main_pos_x"]),
                                                   (p["elev_chord"], p["elev_span"], p["elev_pos_x"])])
        ir_o = strip_added_mass_roll([(old["main_chord"], old["main_span"], -old["main_lateral_offset"]),
                                      (old["main_chord"], old["main_span"], old["main_lateral_offset"]),
                                      (old["elev_chord"], old["elev_span"], 0.0)])
        ir_n = strip_added_mass_roll([(p["main_chord"], p["main_span"], -p["main_lateral_offset"]),
                                      (p["main_chord"], p["main_span"], p["main_lateral_offset"]),
                                      (p["elev_chord"], p["elev_span"], 0.0)])
        auth_h_o = max(old["main_chord"] * old["main_span"] * _foil_eff(old), 1e-6)
        auth_h_n = max(p["main_chord"] * p["main_span"] * _foil_eff(p), 1e-6)
        sh = ((p["mass"] + mh_n) / (old["mass"] + mh_o)) * (auth_h_o / auth_h_n)
        for k in ("kp_heave", "ki_heave", "kd_heave"):
            if k not in ov and abs(sh - 1.0) > 1e-6:
                p[k] = round(old[k] * sh, 4)
        arm_e_o = max(abs(old["elev_pos_x"]) * old["elev_chord"] * old["elev_span"], 1e-6)
        arm_e_n = max(abs(p["elev_pos_x"]) * p["elev_chord"] * p["elev_span"], 1e-6)
        sp_ = ((p["iyy"] + ip_n) / (old["iyy"] + ip_o)) * (arm_e_o / arm_e_n)
        for k in ("kp_pitch", "ki_pitch", "kd_pitch"):
            if k not in ov and abs(sp_ - 1.0) > 1e-6:
                p[k] = round(old[k] * sp_, 5)
        arm_r_o = max(old["main_lateral_offset"] * auth_h_o, 1e-6)
        arm_r_n = max(p["main_lateral_offset"] * auth_h_n, 1e-6)
        sr = ((p["ixx"] + ir_n) / (old["ixx"] + ir_o)) * (arm_r_o / arm_r_n)
        for k in ("kp_roll", "kd_roll"):
            if k not in ov and abs(sr - 1.0) > 1e-6:
                p[k] = round(old[k] * sr, 5)


def apply_design(p: dict, overrides: dict) -> dict:
    """New params dict with the edits applied (the input is not modified)."""
    old = dict(p)
    p = dict(p)
    p.setdefault("_base_mass", old["mass"])
    p.setdefault("_base_inertia", (old["ixx"], old["iyy"], old["izz"]))
    p.setdefault("_base_payloads", list(old.get("default_payloads", [])))
    ov = dict(overrides or {})
    payloads = ov.pop("payloads", None)
    empty_mass = ov.pop("empty_mass", None)
    strut = ov.pop("strut_length", None)
    extra_keys = ("lqr_weights", "gear_rated_torque", "shaft_diameter", "prop_power_max",
                  "prop_tau_s", "dual_height", "gnss_fallback", "empty_mass", "mass_empty", "cg_z")
    for k, v in ov.items():
        if k in p or k in extra_keys:
            if k in p and isinstance(p[k], (int, float)) and not isinstance(p[k], bool) and v is None:
                continue
            p[k] = v
    dim_bounds = {
        "main_chord": (0.02, None), "main_span": (0.05, None),
        "elev_chord": (0.02, None), "elev_span": (0.05, None),
        "main_flap_frac": (0.05, 0.85), "main_flap_span_frac": (0.1, 1.0),
        "hull_length": (0.2, None), "hull_beam": (0.1, None),
        "max_draft": (0.02, None), "target_speed": (0.5, None), "max_thrust": (1.0, None),
    }
    for k, (lo, hi) in dim_bounds.items():
        if k in p and p[k] is not None:
            val = max(float(p[k]), lo)
            p[k] = min(val, hi) if hi is not None else val
    if "main_span" in ov:
        p["main_lateral_offset"] = 0.05 + p["main_span"] / 2
    if strut is not None:
        strut_v = max(float(strut), 0.15)
        dz = p["main_pos_z"] - p["elev_pos_z"]
        p["main_pos_z"] = strut_v - p["hull_offset"]
        p["elev_pos_z"] = p["main_pos_z"] - dz
    if payloads is not None or empty_mass is not None:
        sb = p["hull_beam"] / max(old["hull_beam"], 1e-6)
        sl = p["hull_length"] / max(old["hull_length"], 1e-6)
        mp_ = mass_properties(p, payloads, empty_mass=empty_mass, hull_scale=(sb, sl))
        p["mass"] = mp_["mass"]
        p["mass_empty"] = mp_["mass_empty"]
        p["ixx"], p["iyy"], p["izz"] = [round(float(x), 3) for x in mp_["inertia"]]
        p["cg_offset"] = (mp_["cg"][0], mp_["cg"][1])
        p["cg_z"] = float(mp_["cg"][2])
        if payloads is not None:
            p["default_payloads"] = mp_["payloads"]
    if ov or payloads is not None or empty_mass is not None or strut is not None:
        ov_flags = dict(ov)
        if payloads is not None:
            ov_flags["payloads"] = True
        if empty_mass is not None:
            ov_flags["empty_mass"] = True
        if strut is not None:
            ov_flags["strut_length"] = True
        _scale_inertias_and_hull(old, p, ov_flags)
        _scale_structures_drives_and_gains(old, p, ov_flags)
    if overrides and (TRIM_EDIT_KEYS & set(overrides)):
        _reschedule(p)
    return p


def _trim_flap(p: dict, speed: float):
    from hydrofoil.linearize import trim
    from hydrofoil.vehicle import VehiclePhysics
    vp = VehiclePhysics(dict(p, controller="pid", sensors="ideal", supervisor=False))   # at the actual CG
    try:
        _, u = trim(vp, speed, u0=(5.0, 0.0, 0.0, 200.0), max_iter=80)
        return float(u[0]), float(u[1]), float(u[3])
    except Exception:
        return None


_CURVES: dict = {}
_CONTROL_KEYS = ("lqr_", "est_", "trim_ff", "kp_", "ki_", "kd_", "controller", "sensors", "supervisor", "kf_",
                 "servo_params", "default_payloads", "target_speed", "max_thrust", "phi_ref", "theta_ref",
                 "_base_", "mass_empty", "allocator", "safety_margin", "reverse_drive_enabled",
                 "flap_cmd_max_deg", "wave_type")


def trim_curve(p: dict, speeds=None) -> list[dict]:
    """Level trim at each speed (cached on the physical parameters only: controller settings do not change trim)."""
    warnings.simplefilter("ignore")
    speeds = speeds if speeds is not None else np.round(np.arange(3.0, 16.01, 0.5), 2)   # 16: the CAD boat at 0 deg
    key = (repr(sorted((k, repr(v)) for k, v in p.items() if not k.startswith(_CONTROL_KEYS))), tuple(speeds))
    if key in _CURVES:
        return _CURVES[key]
    out = []
    for s in speeds:
        r = _trim_flap(p, float(s))
        out.append({"speed": float(s), "flap": r and r[0], "elev": r and r[1], "thrust": r and r[2]})
    if len(_CURVES) > 64:
        _CURVES.clear()
    _CURVES[key] = out
    return out


def _reschedule(p: dict):
    """Re-derive the trim feed-forward and the LQR / estimator speed schedule for an edited boat."""
    curve = trim_curve(p)
    ok = [c for c in curve if c["flap"] is not None and abs(c["flap"]) <= FLAP_TRIM_LIMIT]
    if not ok:
        return
    v_lo = ok[0]["speed"]
    v_hi = max(c["speed"] for c in ok)
    ref = min(ok, key=lambda c: abs(c["speed"] - max(v_lo + 1.0, 0.75 * p["target_speed"])))
    p["trim_ff_deg"], p["trim_ff_speed"] = ref["flap"], ref["speed"]
    hi = min(v_hi, max(p["target_speed"] * 1.2, v_lo + 3.0))
    p["lqr_speeds"] = tuple(float(x) for x in np.round(np.linspace(v_lo, hi, 5), 2))
    p["lqr_engage_speed"] = float(v_lo + 0.3)
    p["est_engage_speed"] = float(v_lo)


def design_summary(base_d: dict | None, edit_d: dict, p: dict) -> dict:
    """Plain-English qualitative design assessment for newcomers and operators."""
    v_min = edit_d.get("v_min_foil")
    cf = edit_d.get("cruise_flap")
    margin = edit_d.get("flap_margin")
    thrust = edit_d.get("cruise_thrust")
    max_thr = float(p["max_thrust"])
    depth = float(edit_d.get("foil_depth", 0.0))
    cav = float(edit_d.get("cavitation_number", 9.9))
    ar = float(edit_d.get("aspect_ratio", 0.0))
    cgx = float(edit_d.get("cg_x", 0.0))
    cgy = float(edit_d.get("cg_y", 0.0))
    cgz = float(edit_d.get("cg_z", 0.0))

    issues, cautions = [], []
    if v_min is None or cf is None or margin is None:
        issues.append("the boat cannot find a level trim at cruise speed with these weights/foils")
    else:
        lim_d = float(p.get("flap_cmd_max_deg", 20.0))
        if margin < 5.0:
            issues.append(f"only {margin:.1f}° of flap travel remains at cruise before hitting ±{lim_d:.0f}° limits")
        elif margin < 8.0:
            cautions.append(f"flap headroom is modest ({margin:.1f}°) in steep turns or waves")
        if v_min > float(edit_d["cruise_speed"]) * 0.9:
            cautions.append(f"cruise speed ({edit_d['cruise_speed']:.1f} m/s) sits close to the {v_min:.1f} m/s minimum foil speed")
    if depth <= 0.15:
        issues.append(f"main foil is only {depth:.2f} m deep at flying height and will breach the surface")
    elif depth < 0.35:
        cautions.append(f"shallow foil submergence ({depth:.2f} m) limits safe bank angle in waves")
    if thrust is not None and thrust > 0.92 * max_thr:
        issues.append(f"cruise thrust ({thrust:.0f} N) leaves almost no margin under the {max_thr:.0f} N motor limit for takeoff")
    elif thrust is not None and thrust > 0.65 * max_thr:
        cautions.append(f"cruise uses {100 * thrust / max(max_thr, 1e-6):.0f}% of max thrust; hump acceleration may be slow")
    if cav < 1.3:
        issues.append(f"cavitation number σ={cav:.2f} indicates likely blade cavitation at cruise speed")
    elif cav < 2.0:
        cautions.append(f"cavitation margin is getting thin (σ={cav:.2f})")
    if abs(cgy) > 0.10:
        cautions.append(f"sideways CG offset ({cgy:+.2f} m) forces unequal flap lift even when flying straight")

    if issues:
        verdict = {"status": "danger", "headline": "Needs adjustment before flying — " + "; ".join(issues) + "."}
    elif cautions:
        verdict = {"status": "caution", "headline": "Flyable with narrower margins — " + "; ".join(cautions) + "."}
    else:
        verdict = {"status": "good",
                   "headline": (f"Balanced and ready to fly: takes off near {v_min:.1f} m/s, cruises at "
                                f"{edit_d['cruise_speed']:.1f} m/s with {margin:.1f}° spare flap travel.")}

    fwd_txt = ("well balanced fore/aft" if abs(cgx) <= 0.03 else
               f"{abs(cgx) * 100:.0f} cm nose-heavy (front wing carries extra lift; tail works harder to hold nose up)" if cgx > 0 else
               f"{abs(cgx) * 100:.0f} cm tail-heavy (rear elevator carries more load; watch pitch sensitivity)")
    lat_txt = ("symmetrical left/right" if abs(cgy) <= 0.015 else
               f"pulled {abs(cgy) * 100:.0f} cm to {'starboard' if cgy > 0 else 'port'} (front split flaps trim asymmetrically in straight flight)")
    if abs(cgz) > 0.02:
        z_txt = f" Vertical CG shifted {cgz:+.2f} m (updates pitch/roll inertias Ixx/Iyy and vertical foil/hull/prop lever arms)."
    else:
        z_txt = ""
    inertia_txt = ""
    if base_d is not None and base_d.get("iyy") and edit_d.get("iyy"):
        d_ixx = 100.0 * (edit_d["ixx"] - base_d["ixx"]) / max(base_d["ixx"], 1e-6)
        d_iyy = 100.0 * (edit_d["iyy"] - base_d["iyy"]) / max(base_d["iyy"], 1e-6)
        inertia_txt = f" Roll inertia {d_ixx:+.0f}%, pitch inertia {d_iyy:+.0f}% vs preset."
    balance = f"Center of gravity is {fwd_txt} and {lat_txt}.{z_txt}{inertia_txt}"

    ar_txt = ("high-aspect narrow wing (efficient low drag, but outer tips sit closer to breaching in steep turns)"
              if ar >= 5.5 else
              "compact lower-aspect wing (slightly higher induced drag, more tip clearance in turns)"
              if ar <= 3.0 else
              "moderate-aspect wing (good balance of efficiency and tip clearance)")
    wings = (f"{edit_d['wing_area']:.3f} m² main wing with AR {ar:.2f} per half — {ar_txt}. "
             + (f"Slowest clean foil speed is ~{v_min:.1f} m/s." if v_min else "Cannot maintain level lift across the speed sweep."))

    thr_str = f"{thrust:.0f} N ({edit_d['cruise_power_kw']:.2f} kW hydro power)" if thrust is not None else "unavailable"
    cav_str = "safe margin from cavitation bubbles" if cav >= 2.2 else ("tight cavitation margin" if cav >= 1.5 else "high cavitation risk")
    power = f"Estimated cruise drag/thrust is {thr_str} vs {max_thr:.0f} N max thrust; σ={cav:.2f} ({cav_str}) at {depth:.2f} m foil depth."

    return {"verdict": verdict, "balance": balance, "wings": wings, "power": power}


def derived(p: dict, include_summary: bool = True) -> dict:
    """What the design does, without flying it: wing loading, takeoff and cruise trim, power, margins."""
    warnings.simplefilter("ignore")
    mc = max(float(p["main_chord"]), 1e-6)
    ms = max(float(p["main_span"]), 1e-6)
    S = max(2 * mc * ms, 1e-6)
    W = p["mass"] * G
    curve = trim_curve(p)
    ok = [c for c in curve if c["flap"] is not None and abs(c["flap"]) <= FLAP_TRIM_LIMIT]
    cruise = _trim_flap(p, p["target_speed"])
    depth = (p["hull_offset"] - p["flying_height"]) + p["main_pos_z"]
    q = 0.5 * RHO_WATER * max(float(p["target_speed"]), 0.1) ** 2
    cg = p.get("cg_offset", (0.0, 0.0))
    m_def = sum(q_i["mass"] for q_i in _sanitize_payloads(p.get("default_payloads", [])))
    m_empty = float(p.get("mass_empty", max(float(p["mass"]) - m_def, 0.0)))
    out = {
        "mass": p["mass"], "mass_empty": m_empty,
        "cg_x": cg[0], "cg_y": cg[1], "cg_z": float(p.get("cg_z", 0.0)),
        "ixx": float(p["ixx"]), "iyy": float(p["iyy"]), "izz": float(p["izz"]),
        "wing_area": S, "aspect_ratio": ms / mc, "wing_loading": W / S,
        "foil_depth": depth, "strut_length": p["hull_offset"] + p["main_pos_z"],
        "v_min_foil": ok[0]["speed"] if ok else None, "v_max_trim": max(c["speed"] for c in ok) if ok else None,
        "cruise_speed": p["target_speed"],
        "cruise_flap": cruise and cruise[0], "cruise_elev": cruise and cruise[1],
        "cruise_thrust": cruise and cruise[2], "cruise_power_kw": cruise and cruise[2] * p["target_speed"] / 1000,
        "flap_margin": cruise and float(p.get("flap_cmd_max_deg", 20.0)) - abs(cruise[0]),
        "cavitation_number": (101325 + RHO_WATER * G * depth - 2340) / q,
        "reynolds": p["target_speed"] * p["main_chord"] / 1.05e-6,
        "curve": curve,
    }
    if include_summary:
        out["summary"] = design_summary(None, out, p)
    return out
