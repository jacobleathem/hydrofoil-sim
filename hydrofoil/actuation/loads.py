"""
Per-surface actuator loads from a simulated trajectory.

The flight controller commands three VIRTUAL channels (delta_main, delta_roll, delta_elev); the
hardware has three PHYSICAL drives: left flap = main - roll, right flap = main + roll, elevator.
Loads, limits and saturation belong to the physical drives, so everything here is per drive.
"""

from __future__ import annotations

import numpy as np

from hydrofoil.actuation.hinge import HingeMoment, SurfaceGeometry
from hydrofoil.forces.foil import free_surface_lift_factor

DRIVES = ("left", "right", "elev")


def surface_models(p: dict) -> dict[str, HingeMoment]:
    flap_span = p["main_span"] * (p.get("main_flap_span_frac", 1.0) if p["main_actuation"] == "flap" else 1.0)
    c_flap = p.get("main_flap_chord")
    c_main = float(c_flap) / max(float(p["main_flap_frac"]), 1e-4) if (c_flap is not None and float(c_flap) > 0.0) else p["main_chord"]
    main = SurfaceGeometry(chord=c_main, span=flap_span, layout=p["main_actuation"],
                           flap_frac=p["main_flap_frac"], pivot_frac=p["main_pivot_frac"],
                           hinge_setback=p.get("main_flap_setback", 0.0),
                           j_struct=p["main_surface_inertia"])
    elev = SurfaceGeometry(chord=p["elev_chord"], span=p["elev_span"], layout=p["elev_actuation"],
                           pivot_frac=p["elev_pivot_frac"], j_struct=p["elev_surface_inertia"])
    return {"left": HingeMoment(main), "right": HingeMoment(main), "elev": HingeMoment(elev)}


def drive_angles(cmds: dict) -> dict[str, float]:
    """Physical surface angles [deg] from explicit flap commands or virtual channels."""
    left = float(cmds["delta_left"]) if "delta_left" in cmds else float(cmds["delta_main"] - cmds["delta_roll"])
    right = float(cmds["delta_right"]) if "delta_right" in cmds else float(cmds["delta_main"] + cmds["delta_roll"])
    return {"left": left, "right": right, "elev": float(cmds["delta_elev"])}


class _Flow:
    """Steady hinge moment of one drive as a function of its deflection, at a frozen flow."""

    def __init__(self, hm: HingeMoment, foil, V: float, alpha: float, lf: float):
        self.hm, self.foil, self.V, self.alpha, self.lf = hm, foil, V, alpha, lf

    def __call__(self, delta_rad: float) -> float:
        hm = self.hm
        if hm.g.layout == "flap":
            return hm.steady(self.V, self.alpha, delta_rad, lift_factor=self.lf)
        # all-moving: the deflection IS angle of attack (times the foil's effectiveness)
        cl = float(self.foil.tables.cl(np.degrees(self.alpha) + self.foil.flap_effectiveness * np.degrees(delta_rad))) * self.lf
        return hm.steady(self.V, self.alpha, delta_rad, cl=cl)


def flow_models(vp, state, env, hinges, r_overrides: dict | None = None) -> dict[str, _Flow]:
    """Per drive: the flow seen by its surface (relative velocity incl. waves, free-surface and 3-D factors)."""
    eta, nu = state[:6], state[6:]
    foils = {"left": vp.left_foil, "right": vp.right_foil, "elev": vp.elev_foil}
    out = {}
    for name, foil in foils.items():
        r_use = None if r_overrides is None else r_overrides.get(name)
        nu_rel = nu.copy()
        nu_rel[:3] -= env.foil_vel_body[name]
        V, alpha = foil.flow(nu_rel, r_attach=r_use)
        if name == "elev":
            alpha -= vp._eps_used                      # the tail flies in the wing's downwash
        try:
            h_c = vp._submergence_over_chord(foil, eta, env.foil_surface_z[name], r_attach=r_use)
        except TypeError:
            h_c = vp._submergence_over_chord(foil, eta, env.foil_surface_z[name])
        lf = foil.lift_slope_factor * free_surface_lift_factor(h_c)
        out[name] = _Flow(hinges[name], foil, V, alpha, lf)
    return out


def steady_moments(vp, state, cmds, env, hinges) -> dict[str, float]:
    """Steady hydrodynamic hinge moment on each drive [N m] at one instant (+ = TE down)."""
    ang = drive_angles(cmds)
    out = {}
    for name, f in flow_models(vp, state, env, hinges).items():
        out[name] = f(np.radians(ang[name]))
        out[name + "_V"] = f.V
    return out


def output_torque(hm: HingeMoment, V, delta, m_steady, dt: float, friction: float):
    """
    Torque the drive must apply at the output shaft [N m] along a sampled trajectory.
    delta in rad (array), m_steady the steady hydrodynamic moment (array), V flow speed (array).
    Returns (tau_out, d_dot, d_ddot).
    """
    d_dot = np.gradient(delta, dt)
    d_ddot = np.gradient(d_dot, dt)
    j = hm.g.j_struct + hm.i_added
    b = np.array([hm.damping(v) for v in np.atleast_1d(V)])
    tau = j * d_ddot + b * d_dot + friction * np.sign(d_dot) - m_steady
    return tau, d_dot, d_ddot
