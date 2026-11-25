"""
Physics-constrained control allocation for independent split front flaps + single rear elevator.

Maps the three virtual controller commands (delta_main, delta_elev, delta_roll) onto the three
physical drives (left, right, elev) via an angle-normalized 4x3 local Jacobian and an exact
3-variable box-constrained Quadratic Program (QP):

    min_{du}  0.5 || W (B du - dw_des) ||^2 + 0.5 reg || du - du_nom ||^2
    s.t.      step_lo <= u_cur + du <= step_hi

Three coupled safety layers cap both the allowed physical flap angles and any reverse-drive split:
  Layer 1 (Physics / Hardware): mechanical stops, asymmetric local-AoA stall around wing
      incidence, V^2 servo hinge-torque limits, speed/depth cavitation number, and slew rate.
  Layer 2 (Control Stability Reserves): minimum net lift floor (gamma_min * m*g / cos(phi))
      and rear elevator pitch reserve; sheds reverse-drive split first while locking Heave & Pitch.
  Layer 3 (Safety Margin Preset): "conservative" | "standard" | "sport" scaling of disturbance
      reserves (never permitted to relax Layer 1 or Layer 2 hard limits).
"""

from __future__ import annotations

import math
import time
from typing import Any

import numpy as np

from hydrofoil.actuation.loads import flow_models
from hydrofoil.actuation.servo import DEFAULT_SERVO_PARAMS
from hydrofoil.constants import G
from hydrofoil.forces.foil import cavitation_number, cp_min_estimate

DRIVES = ("left", "right", "elev")
_RAD1 = math.pi / 180.0
_DEG_PER_RAD = 180.0 / math.pi


def _clip(x: float, a: float, b: float) -> float:
    return a if x < a else (b if x > b else x)

SAFETY_PRESETS: dict[str, dict[str, float]] = {
    "conservative": {
        "stall_margin_deg": 3.5,
        "torque_util_max": 0.75,
        "cav_margin": 1.50,
        "lift_floor_frac": 0.95,
        "elev_reserve_min": 0.35,
        "sideslip_max_deg": 5.5,
        "lat_g_max": 0.22,
        "reverse_drive_max_split_deg": 16.0,
        "bank_scale": 0.80,
    },
    "standard": {
        "stall_margin_deg": 2.5,
        "torque_util_max": 0.85,
        "cav_margin": 1.25,
        "lift_floor_frac": 0.90,
        "elev_reserve_min": 0.25,
        "sideslip_max_deg": 6.5,
        "lat_g_max": 0.30,
        "reverse_drive_max_split_deg": 24.0,
        "bank_scale": 1.00,
    },
    "sport": {
        "stall_margin_deg": 1.5,
        "torque_util_max": 0.92,
        "cav_margin": 1.10,
        "lift_floor_frac": 0.85,
        "elev_reserve_min": 0.18,
        "sideslip_max_deg": 7.2,
        "lat_g_max": 0.40,
        "reverse_drive_max_split_deg": 30.0,
        "bank_scale": 1.05,
    },
}


def safety_preset_config(name: str | None) -> tuple[str, dict[str, float]]:
    """Return canonical preset name and its reserve configuration dict."""
    key = str(name or "standard").strip().lower()
    if key not in SAFETY_PRESETS:
        key = "standard"
    return key, SAFETY_PRESETS[key]


def _solve_sym_2(a00, a01, a11, b0, b1):
    det = a00 * a11 - a01 * a01
    if abs(det) <= 1e-18:
        return None
    inv = 1.0 / det
    return (a11 * b0 - a01 * b1) * inv, (a00 * b1 - a01 * b0) * inv


def _solve_sym_3(H, rhs):
    a00, a01, a02 = H[0][0], H[0][1], H[0][2]
    a11, a12, a22 = H[1][1], H[1][2], H[2][2]
    c00 = a11 * a22 - a12 * a12
    c01 = a02 * a12 - a01 * a22
    c02 = a01 * a12 - a02 * a11
    det = a00 * c00 + a01 * c01 + a02 * c02
    if abs(det) <= 1e-18:
        return None
    inv = 1.0 / det
    c11 = a00 * a22 - a02 * a02
    c12 = a01 * a02 - a00 * a12
    c22 = a00 * a11 - a01 * a01
    r0, r1, r2 = rhs[0], rhs[1], rhs[2]
    return [
        (c00 * r0 + c01 * r1 + c02 * r2) * inv,
        (c01 * r0 + c11 * r1 + c12 * r2) * inv,
        (c02 * r0 + c12 * r1 + c22 * r2) * inv,
    ]


def _solve_box_qp_list(Hm: list[list[float]], gv: list[float], lov: list[float], hiv: list[float], x0v=None) -> list[float]:
    """Zero-NumPy-allocation 3-variable box QP solver."""
    h00, h01, h02 = Hm[0][0], Hm[0][1], Hm[0][2]
    h11, h12, h22 = Hm[1][1], Hm[1][2], Hm[2][2]
    scale = max((h00 + h11 + h22) / 3.0, 1.0)
    reg = 1e-8 * scale
    h00 += reg; h11 += reg; h22 += reg
    H_r = [[h00, h01, h02], [h01, h11, h12], [h02, h12, h22]]
    hiv = [hiv[i] if hiv[i] >= lov[i] else lov[i] for i in range(3)]

    if max(hiv[0] - lov[0], hiv[1] - lov[1], hiv[2] - lov[2]) <= 1e-12:
        return [0.5 * (lov[i] + hiv[i]) for i in range(3)]

    xu = _solve_sym_3(H_r, [-gv[0], -gv[1], -gv[2]])
    if xu is not None and lov[0] - 1e-12 <= xu[0] <= hiv[0] + 1e-12 and lov[1] - 1e-12 <= xu[1] <= hiv[1] + 1e-12 and lov[2] - 1e-12 <= xu[2] <= hiv[2] + 1e-12:
        return [_clip(xu[i], lov[i], hiv[i]) for i in range(3)]

    def _clamp3(v):
        return [_clip(v[i], lov[i], hiv[i]) for i in range(3)]

    x = _clamp3(xu if xu is not None else ([0.0, 0.0, 0.0] if x0v is None else x0v))
    tol = 1e-9 * scale
    diag = [max(H_r[i][i], 1e-12) for i in range(3)]
    state = [0, 0, 0]

    def _solve_face(st):
        xf = [lov[i] if st[i] == -1 else (hiv[i] if st[i] == 1 else 0.0) for i in range(3)]
        free = [i for i in range(3) if st[i] == 0]
        nf = len(free)
        if nf == 3:
            return _solve_sym_3(H_r, [-gv[0], -gv[1], -gv[2]])
        if nf == 2:
            i, j = free
            k = 3 - i - j
            sol2 = _solve_sym_2(H_r[i][i], H_r[i][j], H_r[j][j], -gv[i] - H_r[i][k] * xf[k], -gv[j] - H_r[j][k] * xf[k])
            if sol2 is None:
                return None
            xf[i], xf[j] = sol2
        elif nf == 1:
            i = free[0]
            rhs = -gv[i] - sum(H_r[i][k] * xf[k] for k in range(3) if k != i)
            if H_r[i][i] <= 1e-18:
                return None
            xf[i] = rhs / H_r[i][i]
        return xf

    for _ in range(10):
        grad = [H_r[i][0] * x[0] + H_r[i][1] * x[1] + H_r[i][2] * x[2] + gv[i] for i in range(3)]
        ns = [-1 if (x[i] - grad[i] / diag[i]) <= lov[i] + 1e-12 else (1 if (x[i] - grad[i] / diag[i]) >= hiv[i] - 1e-12 else 0) for i in range(3)]
        if ns == state and all((grad[i] >= -tol if state[i] == -1 else (grad[i] <= tol if state[i] == 1 else abs(grad[i]) <= tol)) for i in range(3)):
            if all(lov[i] - 1e-9 <= x[i] <= hiv[i] + 1e-9 for i in range(3)):
                return _clamp3(x)
        state = ns
        xt = _solve_face(state)
        if xt is None:
            xt = x
        if all((lov[i] - 1e-9 <= xt[i] <= hiv[i] + 1e-9) for i in range(3) if state[i] == 0):
            xc = _clamp3(xt)
            gc = [H_r[i][0] * xc[0] + H_r[i][1] * xc[1] + H_r[i][2] * xc[2] + gv[i] for i in range(3)]
            if all((gc[i] >= -tol if state[i] == -1 else (gc[i] <= tol if state[i] == 1 else abs(gc[i]) <= tol)) for i in range(3)):
                return xc
        x = _clamp3(xt)

    best_val = math.inf
    best_x = x
    for s0 in (-1, 0, 1):
        for s1 in (-1, 0, 1):
            for s2 in (-1, 0, 1):
                st = (s0, s1, s2)
                xf = _solve_face(st)
                if xf is None:
                    continue
                if any((xf[i] < lov[i] - 1e-7 or xf[i] > hiv[i] + 1e-7) for i in range(3) if st[i] == 0):
                    continue
                xc = _clamp3(xf)
                val = 0.5 * sum(xc[i] * (H_r[i][0] * xc[0] + H_r[i][1] * xc[1] + H_r[i][2] * xc[2]) for i in range(3)) + sum(gv[i] * xc[i] for i in range(3))
                if val < best_val:
                    best_val = val
                    best_x = xc
    return best_x


def solve_box_qp_3d(
    H: np.ndarray,
    g: np.ndarray,
    lo: np.ndarray,
    hi: np.ndarray,
    x0: np.ndarray | None = None,
) -> np.ndarray:
    """
    Solve the strictly convex 3-variable box-constrained quadratic program:
        min  0.5 x^T H x + g^T x   s.t.  lo <= x <= hi
    Fast scalar-Cramer PDAS solver with exact 27-face fallback.
    """
    Hm = [[float(H[0, 0]), 0.5 * float(H[0, 1] + H[1, 0]), 0.5 * float(H[0, 2] + H[2, 0])],
          [0.0, float(H[1, 1]), 0.5 * float(H[1, 2] + H[2, 1])],
          [0.0, 0.0, float(H[2, 2])]]
    Hm[1][0] = Hm[0][1]; Hm[2][0] = Hm[0][2]; Hm[2][1] = Hm[1][2]
    gv = [float(g[0]), float(g[1]), float(g[2])]
    lov = [float(lo[0]), float(lo[1]), float(lo[2])]
    hiv = [float(hi[0]), float(hi[1]), float(hi[2])]
    x0v = None if x0 is None else [float(x0[0]), float(x0[1]), float(x0[2])]
    return np.array(_solve_box_qp_list(Hm, gv, lov, hiv, x0v), dtype=float)


def _cavitation_delta_bounds(
    aoa_flow_deg: float,
    tau_eff: float,
    sigma: float,
    cav_margin: float,
    d_lo: float,
    d_hi: float,
    is_flap: bool = True,
) -> tuple[float, float]:
    """Closed-form quadratic bounds for cp_min_estimate(aoa_flow + tau*d, d) <= sigma / cav_margin."""
    if sigma >= 50.0 or tau_eff <= 1e-6:
        return d_lo, d_hi
    cp_limit = sigma / max(cav_margin, 1.0)
    c_d = 0.03 if is_flap else 0.0
    a0 = float(aoa_flow_deg)
    t = float(tau_eff)
    d_a0 = -a0 / t

    # Break [d_lo, d_hi] at the only slope discontinuities (d = d_a0, d = 0)
    cuts = [d_lo, d_hi]
    if d_lo < d_a0 < d_hi:
        cuts.append(d_a0)
    if d_lo < 0.0 < d_hi:
        cuts.append(0.0)
    cuts = sorted(set(cuts))

    best_lo, best_hi = None, None
    A = 0.012 * t * t
    inv_2A = 1.0 / (2.0 * A)
    for k in range(len(cuts) - 1):
        seg_a, seg_b = cuts[k], cuts[k + 1]
        if seg_b - seg_a <= 1e-12:
            continue
        mid = 0.5 * (seg_a + seg_b)
        s_a = 1.0 if (a0 + t * mid) >= 0.0 else -1.0
        s_d = 1.0 if mid >= 0.0 else -1.0
        B = 0.024 * t * a0 + 0.12 * s_a * t + c_d * s_d
        C = 0.35 + 0.12 * s_a * a0 + 0.012 * a0 * a0 - cp_limit
        disc = B * B - 4.0 * A * C
        if disc < 0.0:
            continue
        sq = math.sqrt(disc)
        r1 = (-B - sq) * inv_2A
        r2 = (-B + sq) * inv_2A
        lo_i = max(seg_a, r1)
        hi_i = min(seg_b, r2)
        if lo_i <= hi_i + 1e-12:
            best_lo = lo_i if best_lo is None else min(best_lo, lo_i)
            best_hi = hi_i if best_hi is None else max(best_hi, hi_i)

    if best_lo is None:
        d_mid = float(np.clip(d_a0, d_lo, d_hi))
        return d_mid, d_mid
    return float(max(d_lo, best_lo)), float(min(d_hi, best_hi))


def compute_flap_safe_bounds(
    vp,
    state: np.ndarray,
    env,
    prev_deg: dict[str, float],
    preset_cfg: dict[str, float],
    jam: tuple[str, float] | None = None,
    flows: dict | None = None,
    local_info: dict[str, dict] | None = None,
    use_truth: bool = True,
) -> dict[str, Any]:
    """
    Compute Layer 1 (physical) + Layer 2 (slew/jam) safe deflection windows [deg] for left, right, elev.

    Returns dict with:
      envelope_bounds: {name: (lo, hi)} before slew limit (true aerodynamic/hardware safe range)
      step_bounds:     {name: (lo, hi)} intersected with cmd_slew_deg_s * dt
      binding:         {name: constraint_tag}
      sigma_min:       minimum cavitation number across submerged surfaces
    """
    p = vp.params
    eta, nu = state[:6], state[6:]
    dt = float(p["dt"])
    cmd_max_main = float(p.get("flap_cmd_max_deg", 20.0))
    cmd_max_elev = float(p.get("elev_cmd_max_deg", p.get("elev_stop_deg", 20.0)))
    foils = {"left": vp.left_foil, "right": vp.right_foil, "elev": vp.elev_foil}
    r_ovr = None if use_truth else getattr(vp, "_r_nominal", None)
    if flows is None and getattr(vp, "hinges", None):
        flows = flow_models(vp, state, env, vp.hinges, r_overrides=r_ovr)

    env_bounds: dict[str, tuple[float, float]] = {}
    step_bounds: dict[str, tuple[float, float]] = {}
    binding: dict[str, str] = {}
    sigma_min = 99.0

    slew = p.get("cmd_slew_deg_s")
    slew_step = float(slew) * dt if slew else None

    for n in DRIVES:
        foil = foils[n]
        stop = float(p["elev_stop_deg"] if n == "elev" else p["flap_stop_deg"])
        cmd_lim = min(stop, cmd_max_elev if n == "elev" else cmd_max_main)

        if jam is not None and jam[0] == n:
            a_jam = float(np.degrees(vp.drives[n].delta)) if n in vp.drives else float(jam[1])
            env_bounds[n] = (a_jam, a_jam)
            step_bounds[n] = (a_jam, a_jam)
            binding[n] = "jammed"
            continue

        cands: list[tuple[str, float, float]] = [
            ("mech stop", -stop, stop),
            ("cmd envelope", -cmd_lim, cmd_lim),
        ]

        info = local_info.get(n) if local_info else None
        speed = float(info["speed"]) if info is not None else float(abs(nu[0]))
        if info is not None:
            h_c = float(info["submergence_hc"])
        elif r_ovr is not None and n in r_ovr:
            h_c = float(vp._effective_submergence_over_chord(foil, eta, env.foil_surface_z[n], r_attach=r_ovr[n]))
        else:
            h_c = float(vp._effective_submergence_over_chord(foil, eta, env.foil_surface_z[n]))
        aoa_flow = float(info["aoa_flow_deg"]) if info is not None else float(foil.geometry.aoa_offset)
        sigma = float(info["sigma"]) if info is not None else cavitation_number(speed, max(h_c, 0.0) * foil.geometry.chord)
        if h_c > 0.0 and speed >= 2.0:
            sigma_min = min(sigma_min, sigma)

        # Asymmetric local-AoA stall bound (polar peak - safety margin)
        if speed >= 2.0 and h_c > 0.05:
            aoa_peak = float(getattr(foil.tables, "aoa_peak_deg", 14.4))
            aoa_safe = max(2.0, aoa_peak - preset_cfg["stall_margin_deg"])
            tau = max(float(foil.flap_effectiveness), 1e-4)
            # Only apply stall clamp when the surface is not all-moving elevator in takeoff transient,
            # or apply smoothly with minimum 6 deg authority outside extreme flow angles
            s_lo = (-aoa_safe - aoa_flow) / tau
            s_hi = (aoa_safe - aoa_flow) / tau
            if s_lo > -stop or s_hi < stop:
                if s_lo > s_hi:
                    mid = float(np.clip(-aoa_flow / tau, -stop, stop))
                    s_lo, s_hi = mid, mid
                cands.append(("stall margin", s_lo, s_hi))

        # Speed-dependent servo hinge torque cap (uses configured firmware cap so a limp hardware fault
        # is isolated by Supervisor._detect rather than pre-clamping commands)
        if vp.actuator_model == "servo" and flows and n in flows and speed >= 1.5 and h_c > 0.05:
            fl = flows[n]
            cfg_cap = float((p.get("servo_params") or {}).get("torque_cap", DEFAULT_SERVO_PARAMS["torque_cap"]))
            m0 = float(fl(0.0))
            if fl.hm.g.layout == "flap":
                kh = float(fl(_RAD1) - m0)
            else:
                kh = float(fl(_RAD1) - fl(-_RAD1)) * 0.5   # N*m / deg
            m_allow = preset_cfg["torque_util_max"] * cfg_cap
            if abs(kh) > 1e-4:
                d1 = (-m_allow - m0) / kh
                d2 = (m_allow - m0) / kh
                cands.append(("hinge torque", min(d1, d2), max(d1, d2)))
            elif abs(m0) > m_allow:
                # Constant load exceeds torque allowance; nudge toward zero angle
                cands.append(("hinge torque", -cmd_lim, cmd_lim))

        # Speed- & depth-dependent cavitation ceiling
        if speed >= 2.5 and h_c > 0.05:
            is_flap = float(foil.flap_effectiveness) < 0.99
            cav_lo, cav_hi = _cavitation_delta_bounds(
                aoa_flow, float(foil.flap_effectiveness), sigma, preset_cfg["cav_margin"],
                -stop, stop, is_flap=is_flap,
            )
            cands.append(("cavitation", cav_lo, cav_hi))

        # Intersect candidates robustly
        lo, hi = -stop, stop
        bind_lo, bind_hi = "mech stop", "mech stop"
        for tag, c_lo, c_hi in cands:
            if c_lo > lo + 1e-6:
                lo = c_lo
                bind_lo = tag
            if c_hi < hi - 1e-6:
                hi = c_hi
                bind_hi = tag
        if lo > hi:
            # Conflicting physical limits (e.g. wave slam at extreme speed): center on unload target clamped in stops
            unload = float(np.clip(-aoa_flow / max(float(foil.flap_effectiveness), 1e-4), -cmd_lim, cmd_lim))
            lo = hi = unload
            tag = bind_lo if abs(lo - cands[0][1]) > 0 else bind_hi
            binding[n] = f"{tag} (recovery)"
        else:
            # Choose whichever side has smaller remaining margin relative to mechanical stop / cmd limit
            cur = float(prev_deg.get(n, 0.0))
            margin_lo = abs(cur - lo)
            margin_hi = abs(hi - cur)
            # Prefer reporting physical physics constraint (stall / torque / cavitation) if tighter than cmd_lim
            phys_tags = [b for b in (bind_lo, bind_hi) if b not in ("mech stop", "cmd envelope")]
            if phys_tags and (lo > -cmd_lim + 1e-3 or hi < cmd_lim - 1e-3):
                binding[n] = bind_lo if (bind_lo not in ("mech stop", "cmd envelope") and margin_lo <= margin_hi) else (
                    bind_hi if bind_hi not in ("mech stop", "cmd envelope") else phys_tags[0]
                )
            else:
                binding[n] = bind_lo if margin_lo < margin_hi else bind_hi

        env_bounds[n] = (float(lo), float(hi))
        s_lo, s_hi = lo, hi
        if slew_step is not None:
            pv = float(prev_deg.get(n, 0.0))
            sv_lo, sv_hi = pv - slew_step, pv + slew_step
            if sv_lo <= hi and sv_hi >= lo:
                s_lo = max(lo, sv_lo)
                s_hi = min(hi, sv_hi)
            else:
                # Move one slew step toward the safe envelope rather than violating feasibility
                tgt_near = float(np.clip(pv, lo, hi))
                mv = pv + float(np.clip(tgt_near - pv, -slew_step, slew_step))
                s_lo = s_hi = mv
        step_bounds[n] = (float(min(s_lo, s_hi)), float(max(s_lo, s_hi)))

    return {
        "envelope_bounds": env_bounds,
        "step_bounds": step_bounds,
        "binding": binding,
        "sigma_min": float(sigma_min),
    }


class ControlAllocator:
    """
    Physics-constrained 4x3 Jacobian + 3-variable QP control allocator with stability-gated
    reverse drive for independent left/right front flaps and single-piece rear elevator.
    """

    def __init__(self, params: dict):
        self.params = params
        self.last_status: dict[str, Any] = {}
        self._nu_rel = np.zeros(6)
        self._nu_sym = np.zeros(6)

    def _jacobians(
        self,
        vp,
        state: np.ndarray,
        env,
        u_cur: list[float],
        nominal_nu: np.ndarray | None = None,
        use_truth: bool = True,
    ) -> tuple[list[list[float]], list[list[float]], list[float], dict[str, dict]]:
        """
        Compute live 4x3 wrench Jacobian B (rows: Fz, Mx, My, Mz; cols: L, R, E),
        symmetric nominal Jacobian B_nom, current wing+tail wrench w0, and per-foil info.
        """
        p = vp.params
        eta, nu = state[:6], state[6:]
        foils = (vp.left_foil, vp.right_foil, vp.elev_foil)
        downwash = float(getattr(vp, "_eps_used", 0.0))
        vent = getattr(vp, "_vent", {}) if use_truth else {}
        r_ovr = None if use_truth else getattr(vp, "_r_nominal", None)

        B = [[0.0, 0.0, 0.0] for _ in range(4)]
        w0 = [0.0, 0.0, 0.0, 0.0]
        local_info: dict[str, dict] = {}
        nu_rel = self._nu_rel
        nu0, nu1, nu2 = float(nu[0]), float(nu[1]), float(nu[2])
        nu3, nu4, nu5 = float(nu[3]), float(nu[4]), float(nu[5])
        nu_rel[3], nu_rel[4], nu_rel[5] = nu3, nu4, nu5
        vel_body = env.foil_vel_body

        for j, name in enumerate(DRIVES):
            foil = foils[j]
            r_use = None if r_ovr is None else r_ovr.get(name)
            vb = vel_body[name]
            nu_rel[0] = nu0 - float(vb[0])
            nu_rel[1] = nu1 - float(vb[1])
            nu_rel[2] = nu2 - float(vb[2])
            if r_use is None:
                h_c = vp._effective_submergence_over_chord(foil, eta, env.foil_surface_z[name])
            else:
                h_c = vp._effective_submergence_over_chord(foil, eta, env.foil_surface_z[name], r_attach=r_use)
            dw = downwash if j == 2 else 0.0
            res = foil.local_derivatives(
                nu_rel, delta=u_cur[j], submergence_over_chord=h_c,
                downwash_rad=dw, ventilation=float(vent.get(name, 0.0)), r_attach=r_use,
            )
            local_info[name] = res
            dtau = res["dtau_ddelta"]
            tau = res["tau"]
            for r in range(4):
                B[r][j] = float(dtau[r + 2])
                w0[r] += float(tau[r + 2])

        B_unc = [row[:] for row in B]
        k_cc = _clip(float(p.get("split_flap_cross_coupling", 0.0)), 0.0, 0.35)
        if k_cc > 0.0:
            omk = 1.0 - k_cc
            for r in range(4):
                b0, b1 = B[r][0], B[r][1]
                B[r][0] = omk * b0 + k_cc * b1
                B[r][1] = k_cc * b0 + omk * b1

        # Reuse uncoupled B when flow, depth, angles, and ventilation are symmetric.
        vb_l, vb_r, vb_e = vel_body["left"], vel_body["right"], vel_body["elev"]
        if (nominal_nu is None
                and abs(u_cur[0] - u_cur[1]) < 0.05
                and abs(local_info["left"]["submergence_hc"] - local_info["right"]["submergence_hc"]) < 0.02
                and abs(nu1) + abs(nu3) + abs(nu5) < 0.01
                and float(vent.get("left", 0.0)) == 0.0 and float(vent.get("right", 0.0)) == 0.0
                and max(abs(float(vb_l[0])), abs(float(vb_l[1])), abs(float(vb_l[2])),
                        abs(float(vb_r[0])), abs(float(vb_r[1])), abs(float(vb_r[2])),
                        abs(float(vb_e[0])), abs(float(vb_e[1])), abs(float(vb_e[2]))) < 1e-4):
            return B, B_unc, w0, local_info

        if nominal_nu is None:
            nu_sym = self._nu_sym
            nu_sym[0] = nu0 if nu0 > 0.1 else 0.1
            nu_sym[1] = 0.0
            nu_sym[2] = nu2
            nu_sym[3] = 0.0
            nu_sym[4] = nu4
            nu_sym[5] = 0.0
        else:
            nu_sym = nominal_nu
        h_mean = 0.5 * (local_info["left"]["submergence_hc"] + local_info["right"]["submergence_hc"])
        d_sym = _clip(0.5 * (u_cur[0] + u_cur[1]), -14.0, 14.0)
        rL_ref = foils[0].r_attach if r_ovr is None else r_ovr.get("left", foils[0].r_attach)
        rR_ref = foils[1].r_attach if r_ovr is None else r_ovr.get("right", foils[1].r_attach)
        rE_ref = foils[2].r_attach if r_ovr is None else r_ovr.get("elev", foils[2].r_attach)
        rL = vp.left_foil.local_derivatives(nu_sym, delta=d_sym, submergence_over_chord=h_mean, r_attach=rL_ref)
        col_L = rL["dtau_ddelta"]
        if (abs(float(rL_ref[1]) + float(rR_ref[1])) < 1e-6
                and abs(float(rL_ref[0]) - float(rR_ref[0])) < 1e-6
                and abs(float(rL_ref[2]) - float(rR_ref[2])) < 1e-6):
            col_R = [float(col_L[2]), -float(col_L[3]), float(col_L[4]), -float(col_L[5])]
        else:
            rR = vp.right_foil.local_derivatives(nu_sym, delta=d_sym, submergence_over_chord=h_mean, r_attach=rR_ref)
            col_R = [float(rR["dtau_ddelta"][r + 2]) for r in range(4)]
        rE = vp.elev_foil.local_derivatives(nu_sym, delta=_clip(u_cur[2], -14.0, 14.0),
                                             submergence_over_chord=local_info["elev"]["submergence_hc"],
                                             downwash_rad=downwash, r_attach=rE_ref)
        col_E = rE["dtau_ddelta"]
        B_nom = [[float(col_L[r + 2]), col_R[r], float(col_E[r + 2])] for r in range(4)]
        return B, B_nom, w0, local_info

    def allocate(
        self,
        vp,
        raw: dict[str, float],
        state: np.ndarray,
        env,
        prev_deg: dict[str, float],
        u_meas: float | None = None,
        jam: tuple[str, float] | None = None,
        flows: dict | None = None,
        use_truth: bool = True,
    ) -> tuple[dict[str, float], dict[str, Any]]:
        """
        Solve the physics-constrained Jacobian QP for [left, right, elev] angles [deg].
        Returns (target_dict, alloc_status).
        """
        t0 = time.perf_counter()
        p = vp.params
        preset_name, cfg = safety_preset_config(p.get("safety_margin", "standard"))
        rev_enabled = bool(p.get("reverse_drive_enabled", True))
        u_eff = max(float(state[6] if u_meas is None else u_meas), 0.0)

        u_nom = [
            float(raw["delta_main"] - raw["delta_roll"]),
            float(raw["delta_main"] + raw["delta_roll"]),
            float(raw["delta_elev"]),
        ]
        if vp.drives:
            u_cur = [
                float(vp.drives["left"].delta) * _DEG_PER_RAD,
                float(vp.drives["right"].delta) * _DEG_PER_RAD,
                float(vp.drives["elev"].delta) * _DEG_PER_RAD,
            ]
        else:
            u_cur = [float(prev_deg.get(n, 0.0)) for n in DRIVES]

        B, B_nom, w0, local_info = self._jacobians(vp, state, env, u_cur, use_truth=use_truth)
        b0_true = (B[0][0], B[0][1], B[0][2])
        # Prevent T-foil non-minimum-phase pitch-for-heave inversion: the rear elevator
        # must govern pitch (My) and never pitch nose-down to chase unmet front-flap heave (Fz).
        B[0][2] = 0.0
        B_nom[0][2] = 0.0
        bounds_info = compute_flap_safe_bounds(
            vp, state, env, prev_deg, cfg, jam=jam, flows=flows, local_info=local_info, use_truth=use_truth
        )
        sb = bounds_info["step_bounds"]
        eb = bounds_info["envelope_bounds"]
        step_lo = [sb[n][0] for n in DRIVES]
        step_hi = [sb[n][1] for n in DRIVES]

        mg = max(float(p["mass"]) * G, 1.0)
        y_lat = max(float(p.get("main_flap_center_y") or p["main_lateral_offset"]), 0.05)
        x_tail = max(abs(float(p["elev_pos_x"])), 0.2)
        inv_s2 = [1.0 / (mg * mg), 1.0 / ((mg * y_lat) ** 2), 1.0 / ((mg * x_tail) ** 2), 1.0 / ((0.25 * mg * max(float(p["main_lateral_offset"]), 0.1)) ** 2)]

        if jam is not None and jam[0] in ("left", "right"):
            jam_idx = 0 if jam[0] == "left" else 1
            ok_idx = 1 - jam_idx
            a_jam = float(vp.drives[jam[0]].delta) * _DEG_PER_RAD if vp.drives else float(jam[1])
            u_nom[jam_idx] = a_jam
            u_nom[ok_idx] = a_jam + (2.0 * float(raw["delta_roll"]) if jam_idx == 0 else -2.0 * float(raw["delta_roll"]))
            w_vec = [0.05, 8.0, 8.0, 0.0]
        else:
            w_vec = [3.5, 2.0, 4.0, 0.05]

        alpha_phys = _clip((u_eff - 1.5) / 1.5, 0.0, 1.0)
        slew = p.get("cmd_slew_deg_s")
        slew_step = float(slew) * float(p["dt"]) if slew else None
        if slew_step is not None:
            u_des = [
                u_nom[j] if (jam is not None and jam[0] == DRIVES[j])
                else _clip(u_nom[j], float(prev_deg.get(DRIVES[j], u_cur[j])) - slew_step,
                           float(prev_deg.get(DRIVES[j], u_cur[j])) + slew_step)
                for j in range(3)
            ]
        else:
            u_des = u_nom
        du_nom = [u_des[j] - u_cur[j] for j in range(3)]
        du_lo = [step_lo[j] - u_cur[j] for j in range(3)]
        du_hi = [step_hi[j] - u_cur[j] for j in range(3)]

        dw_des = [B_nom[r][0] * du_nom[0] + B_nom[r][1] * du_nom[1] + B_nom[r][2] * du_nom[2] for r in range(4)]

        def _solve_qp(w_prio: list[float], lo_b: list[float], hi_b: list[float], reg_ang: float = 1e-4, x0_v=None) -> list[float]:
            we = [w_prio[r] * inv_s2[r] for r in range(4)]
            H_q = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
            g_q = [0.0, 0.0, 0.0]
            for i in range(3):
                for k in range(i, 3):
                    s = we[0] * B[0][i] * B[0][k] + we[1] * B[1][i] * B[1][k] + we[2] * B[2][i] * B[2][k] + we[3] * B[3][i] * B[3][k]
                    if i == k:
                        s += reg_ang
                    H_q[i][k] = H_q[k][i] = s
                g_q[i] = -(we[0] * B[0][i] * dw_des[0] + we[1] * B[1][i] * dw_des[1]
                           + we[2] * B[2][i] * dw_des[2] + we[3] * B[3][i] * dw_des[3]) - reg_ang * du_nom[i]
            du_ang = [_clip(du_nom[i], lo_b[i], hi_b[i]) for i in range(3)]
            du_phys = _solve_box_qp_list(H_q, g_q, lo_b, hi_b, du_ang if x0_v is None else x0_v)
            return [alpha_phys * du_phys[i] + (1.0 - alpha_phys) * du_ang[i] for i in range(3)]

        foils = (vp.left_foil, vp.right_foil, vp.elev_foil)
        vel_body = env.foil_vel_body
        nu = state[6:]
        downwash = float(getattr(vp, "_eps_used", 0.0))
        vent = getattr(vp, "_vent", {}) if use_truth else {}
        r_ovr = None if use_truth else getattr(vp, "_r_nominal", None)
        k_cc = _clip(float(p.get("split_flap_cross_coupling", 0.0)), 0.0, 0.35)

        def _eval_nl(u_cand: list[float]) -> tuple[float, float, float]:
            nu_rel_nl = self._nu_rel
            nu0, nu1, nu2 = float(nu[0]), float(nu[1]), float(nu[2])
            nu_rel_nl[3], nu_rel_nl[4], nu_rel_nl[5] = float(nu[3]), float(nu[4]), float(nu[5])
            if k_cc > 0.0 and abs(u_cand[0] - u_cand[1]) > 1e-6:
                u_eff_l = (1.0 - k_cc) * u_cand[0] + k_cc * u_cand[1]
                u_eff_r = (1.0 - k_cc) * u_cand[1] + k_cc * u_cand[0]
            else:
                u_eff_l, u_eff_r = u_cand[0], u_cand[1]
            u_eval = (u_eff_l, u_eff_r, u_cand[2])
            fz_tot = 0.0
            cls = [0.0, 0.0]
            for j, name in enumerate(DRIVES):
                vb = vel_body[name]
                nu_rel_nl[0] = nu0 - float(vb[0])
                nu_rel_nl[1] = nu1 - float(vb[1])
                nu_rel_nl[2] = nu2 - float(vb[2])
                r_use = None if r_ovr is None else r_ovr.get(name)
                dw = downwash if j == 2 else 0.0
                rj = foils[j]._eval_force_and_deriv(
                    nu_rel_nl, delta=u_eval[j],
                    submergence_over_chord=local_info[name]["submergence_hc"],
                    r_attach=r_use, downwash_rad=dw,
                    ventilation=float(vent.get(name, 0.0)), with_deriv=False,
                )
                fz_tot += float(rj["tau"][2])
                if j < 2:
                    cls[j] = float(rj["CL"])
            return fz_tot, cls[0], cls[1]

        du_sol = _solve_qp(w_vec, du_lo, du_hi)
        u_sol = [_clip(u_cur[i] + du_sol[i], step_lo[i], step_hi[i]) for i in range(3)]

        dl_zero_l = -float(local_info["left"]["aoa_flow_deg"]) / max(float(vp.left_foil.flap_effectiveness), 1e-4)
        dl_zero_r = -float(local_info["right"]["aoa_flow_deg"]) / max(float(vp.right_foil.flap_effectiveness), 1e-4)

        cos_phi = max(abs(math.cos(float(state[3]))), 0.25)
        cl_l0 = float(local_info["left"]["CL"])
        cl_r0 = float(local_info["right"]["CL"])
        need_nl = (
            abs(u_sol[1] - u_sol[0]) > 8.0
            or (u_sol[0] * u_sol[1] < -0.5)
            or (cl_l0 * cl_r0 < 0.003)
            or abs(u_sol[0] - dl_zero_l) < 2.5
            or abs(u_sol[1] - dl_zero_r) < 2.5
            or abs(u_sol[0]) > 12.0 or abs(u_sol[1]) > 12.0
            or max(abs(u_sol[0] - u_cur[0]), abs(u_sol[1] - u_cur[1]), abs(u_sol[2] - u_cur[2])) > 3.0
            or float(local_info["left"]["cp_min"]) > 0.8 * float(local_info["left"]["sigma"])
            or float(local_info["right"]["cp_min"]) > 0.8 * float(local_info["right"]["sigma"])
            or float(vent.get("left", 0.0)) > 0.0
            or float(vent.get("right", 0.0)) > 0.0
            or float(vent.get("elev", 0.0)) > 0.0
        )
        if need_nl:
            fz_pred, cl_l, cl_r = _eval_nl(u_sol)
        else:
            fz_pred = (
                w0[0]
                + b0_true[0] * (u_sol[0] - u_cur[0])
                + b0_true[1] * (u_sol[1] - u_cur[1])
                + b0_true[2] * (u_sol[2] - u_cur[2])
            )
            cl_l, cl_r = cl_l0, cl_r0
        lift_pred = max(0.0, -fz_pred)
        lift_needed = mg / cos_phi
        lift_margin = lift_pred / max(lift_needed, 1.0) if u_eff >= 3.0 else 1.0

        elev_lim = max(abs(eb["elev"][0]), abs(eb["elev"][1]), 1.0)
        elev_reserve = _clip(1.0 - abs(u_sol[2]) / elev_lim, 0.0, 1.0)

        def _check_opp(u_v: list[float], cl0: float, cl1: float) -> bool:
            sp = u_v[1] - u_v[0]
            if abs(sp) <= 1.0:
                return False
            if cl0 * cl1 < -1e-4:
                return True
            return (u_v[0] * u_v[1] < -1.0) and (abs(sp) > 9.0)

        split_deg = u_sol[1] - u_sol[0]
        is_opposing = _check_opp(u_sol, cl_l, cl_r)
        rev_gated = False
        rev_reason = "Inactive (symmetric)"

        if jam is not None:
            rev_reason = f"Jam recovery ({jam[0]} locked at {a_jam:+.1f}°)"
        elif is_opposing or abs(split_deg) > 12.0:
            lift_headroom = max(0.0, lift_margin - cfg["lift_floor_frac"])
            pitch_headroom = max(0.0, elev_reserve - cfg["elev_reserve_min"])
            max_split = float(cfg["reverse_drive_max_split_deg"])

            if not rev_enabled and is_opposing:
                rev_gated = True
                rev_reason = "Disabled by setting"
                if abs(du_hi[0] - du_lo[0]) > 1e-6 and abs(du_hi[1] - du_lo[1]) > 1e-6:
                    du_lo_o, du_hi_o = du_lo[:], du_hi[:]
                    if cl_l + cl_r >= 0.0:
                        du_lo_o[0] = max(du_lo_o[0], min(du_hi_o[0], dl_zero_l - u_cur[0]))
                        du_lo_o[1] = max(du_lo_o[1], min(du_hi_o[1], dl_zero_r - u_cur[1]))
                    else:
                        du_hi_o[0] = min(du_hi_o[0], max(du_lo_o[0], dl_zero_l - u_cur[0]))
                        du_hi_o[1] = min(du_hi_o[1], max(du_lo_o[1], dl_zero_r - u_cur[1]))
                    if abs(split_deg) > 9.0 and u_sol[0] * u_sol[1] < -1.0:
                        m_sym = 0.5 * (u_sol[0] + u_sol[1])
                        sgn_s = 1.0 if split_deg >= 0.0 else -1.0
                        t_l, t_r = m_sym - sgn_s * 4.5, m_sym + sgn_s * 4.5
                        if sgn_s >= 0.0:
                            du_lo_o[0] = max(du_lo_o[0], min(du_hi_o[0], t_l - u_cur[0]))
                            du_hi_o[1] = min(du_hi_o[1], max(du_lo_o[1], t_r - u_cur[1]))
                        else:
                            du_hi_o[0] = min(du_hi_o[0], max(du_lo_o[0], t_l - u_cur[0]))
                            du_lo_o[1] = max(du_lo_o[1], min(du_hi_o[1], t_r - u_cur[1]))
                    du_sol = _solve_qp([6.0, 1.5, 6.0, 0.0], du_lo_o, du_hi_o, x0_v=du_sol)
                    u_sol = [_clip(u_cur[i] + du_sol[i], max(step_lo[i], u_cur[i] + du_lo_o[i]), min(step_hi[i], u_cur[i] + du_hi_o[i])) for i in range(3)]
                    split_deg = u_sol[1] - u_sol[0]
                    fz_pred, cl_l, cl_r = _eval_nl(u_sol)
                    is_opposing = _check_opp(u_sol, cl_l, cl_r)
            elif (u_eff >= 3.5 and (lift_margin < cfg["lift_floor_frac"] or elev_reserve < cfg["elev_reserve_min"])) or abs(split_deg) > max_split:
                rev_gated = True
                if u_eff >= 3.5 and lift_margin < cfg["lift_floor_frac"]:
                    rev_reason = f"Shedding split: low lift margin ({lift_margin:.2f})"
                elif u_eff >= 3.5 and elev_reserve < cfg["elev_reserve_min"]:
                    rev_reason = f"Shedding split: pitch reserve ({elev_reserve:.0%})"
                else:
                    rev_reason = f"Capped at {max_split:.0f} deg max split"
                cap_split = max_split * min(1.0, max(0.25, min(lift_headroom / 0.15, pitch_headroom / 0.25)))
                mid_f = 0.5 * (u_sol[0] + u_sol[1])
                half_c = 0.5 * min(abs(split_deg), cap_split)
                sgn_s = 1.0 if split_deg >= 0.0 else -1.0
                tgt_l, tgt_r = mid_f - sgn_s * half_c, mid_f + sgn_s * half_c
                du_lo_r = du_lo[:]
                du_hi_r = du_hi[:]
                if sgn_s >= 0.0:
                    du_lo_r[0] = max(du_lo_r[0], tgt_l - u_cur[0])
                    du_hi_r[1] = min(du_hi_r[1], tgt_r - u_cur[1])
                else:
                    du_hi_r[0] = min(du_hi_r[0], tgt_l - u_cur[0])
                    du_lo_r[1] = max(du_lo_r[1], tgt_r - u_cur[1])
                du_sol = _solve_qp([6.0, 1.5, 6.0, 0.0], du_lo_r, du_hi_r, x0_v=du_sol)
                u_sol = [_clip(u_cur[i] + du_sol[i], step_lo[i], step_hi[i]) for i in range(3)]
                split_deg = u_sol[1] - u_sol[0]
                fz_pred, cl_l, cl_r = _eval_nl(u_sol)
                is_opposing = _check_opp(u_sol, cl_l, cl_r)
            else:
                rev_reason = "Active (stability verified)"

        dt_ms = (time.perf_counter() - t0) * 1000.0
        target = {"left": u_sol[0], "right": u_sol[1], "elev": u_sol[2]}
        lift_fin = max(0.0, -fz_pred) / max(lift_needed, 1.0) if u_eff >= 3.0 else 1.0
        elev_res_fin = _clip(1.0 - abs(target["elev"]) / elev_lim, 0.0, 1.0)
        main_ach = 0.5 * (target["left"] + target["right"])
        roll_ach = 0.5 * (target["right"] - target["left"])
        sat_flags = {
            "main": abs(main_ach - float(raw["delta_main"])) > 0.15 or (
                (target["left"] <= eb["left"][0] + 0.05 and target["right"] <= eb["right"][0] + 0.05)
                or (target["left"] >= eb["left"][1] - 0.05 and target["right"] >= eb["right"][1] - 0.05)
            ),
            "elev": abs(target["elev"] - float(raw["delta_elev"])) > 0.15 or (
                target["elev"] <= eb["elev"][0] + 0.05 or target["elev"] >= eb["elev"][1] - 0.05
            ),
            "roll": abs(roll_ach - float(raw["delta_roll"])) > 0.15 or bool(rev_gated) or (
                (target["left"] <= eb["left"][0] + 0.05 and target["right"] >= eb["right"][1] - 0.05)
                or (target["left"] >= eb["left"][1] - 0.05 and target["right"] <= eb["right"][0] + 0.05)
            ),
        }

        self.last_status = {
            "active_mode": "jacobian",
            "safety_margin": preset_name,
            "angles": target,
            "bounds": {n: [eb[n][0], eb[n][1]] for n in DRIVES},
            "step_bounds": {n: [sb[n][0], sb[n][1]] for n in DRIVES},
            "binding": bounds_info["binding"],
            "split_deg": target["right"] - target["left"],
            "reverse_drive_enabled": rev_enabled,
            "reverse_drive_active": bool(is_opposing and not rev_gated),
            "reverse_drive_gated": bool(rev_gated),
            "reverse_drive_reason": rev_reason,
            "lift_margin": lift_fin,
            "elev_reserve": elev_res_fin,
            "sigma_min": bounds_info["sigma_min"],
            "cp_min_max": float(max(local_info["left"]["cp_min"], local_info["right"]["cp_min"], local_info["elev"]["cp_min"])),
            "hc_min": float(min(local_info["left"]["submergence_hc"], local_info["right"]["submergence_hc"])),
            "saturated": sat_flags,
            "solve_ms": dt_ms,
        }
        return target, self.last_status
