"""
Canonical vehicle physics, shared by the browser dashboard (dashboard.py)
and the Unity-streaming realtime sim (sim_realtime.py) so they can't drift
apart the way sim_realtime.py fell behind dashboard.py's physics this
session (constant-volume buoyancy, force-fed speed, inert wave config).

Everything here — hull buoyancy-vs-draft, wave-coupled foil flow, closed-loop
surge/thrust control, the differential left/right main foil — is exactly
what dashboard.py's SimRunner._run() had grown, extracted so it has one
source of truth. sim.py's simpler model stays as-is for quick trim/plotting
checks; this module is for anything that needs the full picture.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

from hydrofoil.constants import G, RHO_WATER
from hydrofoil.dynamics.rigid_body import (
    RigidBody, strip_added_mass_heave_pitch, strip_added_mass_roll, strip_added_mass_sway_yaw,
)
from hydrofoil.dynamics.kinematics import J, R_body_to_ned
from hydrofoil.forces.foil import (
    Foil, FoilGeometry, FoilTables, free_surface_lift_factor, cavitation_number,
    MAIN_FOIL_FLAP_EFFECTIVENESS, ELEVATOR_FLAP_EFFECTIVENESS, main_flap_effectiveness, MAIN_FOIL_OSWALD_EFFICIENCY,
)
from hydrofoil.forces.buoyancy import gravity_force
from hydrofoil.forces.hull import HullHydrostatics
from hydrofoil.actuation.hinge import thin_airfoil_coeffs
from hydrofoil.control.pid import HydrofoilController
from hydrofoil.control.surge import SurgeController
from hydrofoil.control.actuator import DEFAULT_ACTUATOR_PARAMS
from hydrofoil.actuation.drive import DriveSpec
from hydrofoil.actuation.loads import surface_models, flow_models
from hydrofoil.actuation.servo import ServoDrive, DEFAULT_SERVO_PARAMS
from hydrofoil.control.allocator import ControlAllocator, safety_preset_config
from hydrofoil.control.supervisor import compute_turn_envelope


# Safety bounds for VehiclePhysics.step() — this drives a real-time interactive
# sim, and rapid multi-axis pilot input (banking while also changing height/
# speed) can excite gyroscopic roll<->yaw<->pitch coupling (real physics —
# Fossen's C_RB matrix — not a bug) faster than the independent SISO PIDs can
# track, which can run away to inf/NaN (verified: reproducible within 15-60s
# of randomized combined stick input, even with bank capped at +-8 deg).
# Proper fix is a coupled/MIMO controller (Phase 3: LQR/MPC per the project
# roadmap). Until then, every step clamps state to physically-plausible
# bounds so a runaway can never reach Unity as NaN — worst case it looks like
# an aggressive, ugly maneuver instead of crashing the receiver.
MAX_ATTITUDE_RAD = np.radians(70.0)   # keeps clear of T_euler's +-90 deg singularity
MAX_LINEAR_VEL = 15.0                  # m/s
MAX_ANGULAR_VEL = 3.0                  # rad/s (~172 deg/s)


@dataclass
class WaveSample:
    """
    Wave environment frozen for one integration step, sampled where it matters:
    at the hull (draft) and at each foil's own world position (relative flow and
    free-surface submergence). Sampling per foil is what lets oblique/beam seas
    load the left and right halves differently and so excite roll.
    """
    elevation: float = 0.0                       # at the body origin [m] (for display)
    hull_surface_z: float = 0.0                  # NED z of the surface at the body origin (display)
    hull_cell_surface_z: object = 0.0            # NED z of the surface under every hull footprint cell (N,) or scalar
    foil_surface_z: dict = field(default_factory=dict)   # name -> NED z of the surface at that foil
    foil_vel_body: dict = field(default_factory=dict)    # name -> wave orbital velocity in body frame [m/s]


class VehiclePhysics:
    """
    Builds the vehicle, foils, and controllers from a params dict, and
    advances the full closed-loop simulation one RK4 step at a time.

    params must contain: mass, ixx, iyy, izz, heave_damping, main_chord,
    main_span, main_pos_x, main_pos_z, main_lateral_offset, elev_chord,
    elev_span, elev_pos_x, elev_pos_z, kp/ki/kd_heave, kp/ki/kd_pitch,
    kp/ki/kd_roll, flying_height, phi_ref, theta_ref, target_speed,
    kp/ki/kd_speed, max_thrust, hull_length, hull_beam, hull_x, max_draft, hull_offset,
    hull_drag_coeff, dt. Optional: actuator_params (defaults to
    DEFAULT_ACTUATOR_PARAMS). params is kept live (not copied) — mutating
    it externally (e.g. a live "stick" update) takes effect on the next step().
    """

    def __init__(self, params: dict):
        from hydrofoil.params import resolve_cad_hull
        self.params = resolve_cad_hull(params)          # pontoons + cockpit box (CAD boat): apply the buoyancy toggle
        p = params

        # Hydrodynamic added mass (heave + pitch) from the vehicle's own
        # lifting surfaces — see dynamics/rigid_body.py::strip_added_mass_heave_pitch.
        # Left/right main-foil halves sit at the same (main_pos_x, main_pos_z)
        # and only differ in lateral offset, which doesn't matter for this
        # heave/pitch-only calculation.
        added_mass_heave, added_inertia_pitch = strip_added_mass_heave_pitch([
            (p["main_chord"], p["main_span"], p["main_pos_x"]),   # left half
            (p["main_chord"], p["main_span"], p["main_pos_x"]),   # right half
            (p["elev_chord"], p["elev_span"], p["elev_pos_x"]),
        ])
        added_inertia_roll = (strip_added_mass_roll([
            (p["main_chord"], p["main_span"], -p["main_lateral_offset"]),
            (p["main_chord"], p["main_span"], p["main_lateral_offset"]),
            (p["elev_chord"], p["elev_span"], 0.0),
        ]) if p.get("roll_added_inertia", True) else 0.0)
        z_fly_nom = p["hull_offset"] - p["flying_height"]
        wet_main_nom = float(np.clip(z_fly_nom + p["main_pos_z"], 0.0, p["hull_offset"] + p["main_pos_z"]))
        wet_elev_nom = float(np.clip(z_fly_nom + p["elev_pos_z"], 0.0, p["hull_offset"] + p["elev_pos_z"]))
        strut_count = max(int(p.get("main_strut_count", 1)), 1)
        strut_y = float(p.get("main_strut_y", 0.0) or 0.0)
        self._dual_front_struts = (strut_count >= 2 and strut_y > 0.0)
        if p.get("roll_added_inertia", True) and self._dual_front_struts:
            added_inertia_roll += strip_added_mass_roll([
                (p["main_strut_chord"], wet_main_nom, -strut_y),
                (p["main_strut_chord"], wet_main_nom, strut_y),
            ])
        if p.get("sway_yaw_added_mass", True):
            added_mass_sway, added_inertia_yaw = strip_added_mass_sway_yaw(
                [(p["main_strut_chord"], wet_main_nom, p["main_pos_x"])] * strut_count
                + [(p["elev_strut_chord"], wet_elev_nom, p["elev_pos_x"])])
        else:
            added_mass_sway, added_inertia_yaw = 0.0, 0.0
        # Distributed hull hydrostatics (cross-referenced against Fossen ch.4 GM_T / GM_L, see docs/15).
        self.hull = HullHydrostatics(p["hull_length"], p["hull_beam"], p["hull_x"],
                                     p["hull_offset"], p["max_draft"],
                                     twin_spacing=float(p.get("hull_twin_spacing", 0.0) or 0.0),
                                     boxes=p.get("hull_boxes") or ())
        self.waterplane_area = self.hull.waterplane_area
        self.rigid_body = RigidBody(
            mass=p["mass"], I_b=np.diag([p["ixx"], p["iyy"], p["izz"]]),
            added_mass_heave=added_mass_heave, added_inertia_pitch=added_inertia_pitch,
            added_inertia_roll=added_inertia_roll, added_mass_sway=added_mass_sway,
            added_inertia_yaw=added_inertia_yaw,
        )

        polar = FoilTables.naca0009()
        cav_on = bool(p.get("cavitation_model", True))
        self.main_flap_eff = (p.get("main_flap_effectiveness")
                              or (main_flap_effectiveness(p["main_flap_frac"], p["main_flap_span_frac"])
                                  if p.get("main_actuation", "flap") == "flap" else MAIN_FOIL_FLAP_EFFECTIVENESS))
        flap_cm = (thin_airfoil_coeffs(p["main_flap_frac"])["cm_delta"] * p["main_flap_span_frac"]
                   * p.get("flap_cm_factor", 0.8)) if p.get("flap_moment", True) else 0.0
        stall_deg = p.get("flap_stall_deg", 15.0) if p.get("main_actuation", "flap") == "flap" else None
        # Part-span flap position: main_flap_center_y [m from the centreline] places the flap's extra force at the
        # flap instead of at the half's centroid (main_lateral_offset). None = the lumped model (no offset).
        yc = p.get("main_flap_center_y")
        flap_dy = (float(p["main_lateral_offset"]) - float(yc)) if (yc is not None and p.get("main_actuation", "flap") == "flap") else 0.0
        self.left_foil = Foil(
            FoilGeometry(chord=p["main_chord"], span=p["main_span"], aoa_offset=p.get("main_incidence_deg", 0.0)),
            polar,
            r_attach=np.array([p["main_pos_x"], -p["main_lateral_offset"], p["main_pos_z"]]),
            flap_effectiveness=self.main_flap_eff,
            oswald_efficiency=MAIN_FOIL_OSWALD_EFFICIENCY, flap_cm_delta=flap_cm, flap_stall_deg=stall_deg,
            cavitation_model=cav_on, flap_dy=flap_dy,
        )
        self.right_foil = Foil(
            FoilGeometry(chord=p["main_chord"], span=p["main_span"], aoa_offset=p.get("main_incidence_deg", 0.0)),
            polar,
            r_attach=np.array([p["main_pos_x"], p["main_lateral_offset"], p["main_pos_z"]]),
            flap_effectiveness=self.main_flap_eff,
            oswald_efficiency=MAIN_FOIL_OSWALD_EFFICIENCY, flap_cm_delta=flap_cm, flap_stall_deg=stall_deg,
            cavitation_model=cav_on, flap_dy=-flap_dy,
        )
        self.elev_foil = Foil(
            FoilGeometry(chord=p["elev_chord"], span=p["elev_span"]),
            polar,
            r_attach=np.array([p["elev_pos_x"], 0.0, p["elev_pos_z"]]),
            flap_effectiveness=ELEVATOR_FLAP_EFFECTIVENESS,
            cavitation_model=cav_on,
        )

        # Vertical struts (hull -> foil), normal axis = body y, so AoA = side-slip and they
        # carry side force; wetted span follows the foil's depth below the surface.
        if self._dual_front_struts:
            self.main_strut_l = Foil(FoilGeometry(chord=p["main_strut_chord"], span=1.0), polar,
                                     r_attach=np.array([p["main_pos_x"], -strut_y, p["main_pos_z"]]),
                                     normal_axis=np.array([0.0, 1.0, 0.0]), cavitation_model=cav_on)
            self.main_strut_r = Foil(FoilGeometry(chord=p["main_strut_chord"], span=1.0), polar,
                                     r_attach=np.array([p["main_pos_x"], strut_y, p["main_pos_z"]]),
                                     normal_axis=np.array([0.0, 1.0, 0.0]), cavitation_model=cav_on)
            self.main_strut = self.main_strut_l
        else:
            self.main_strut = Foil(FoilGeometry(chord=p["main_strut_chord"], span=1.0), polar,
                                   r_attach=np.array([p["main_pos_x"], 0.0, p["main_pos_z"]]),
                                   normal_axis=np.array([0.0, 1.0, 0.0]), cavitation_model=cav_on)
            self.main_strut_l = self.main_strut
            self.main_strut_r = self.main_strut
        self.elev_strut = Foil(FoilGeometry(chord=p["elev_strut_chord"], span=1.0), polar,
                               r_attach=np.array([p["elev_pos_x"], 0.0, p["elev_pos_z"]]),
                               normal_axis=np.array([0.0, 1.0, 0.0]), cavitation_model=cav_on)

        self._r_nominal = {n: f.r_attach.copy() for n, f in self._foils().items()}
        self._hull_offset_nominal = float(p["hull_offset"])
        self._cg_z = float(p.get("cg_z", 0.0))
        self._eta_turn_cache = None
        self.set_cg_offset(p.get("cg_offset", (0.0, 0.0)), cg_z=self._cg_z)

        pid_gains = {
            "heave": {"Kp": p["kp_heave"], "Ki": p["ki_heave"], "Kd": p["kd_heave"]},
            "pitch": {"Kp": p["kp_pitch"], "Ki": p["ki_pitch"], "Kd": p["kd_pitch"]},
            "roll":  {"Kp": p["kp_roll"],  "Ki": p["ki_roll"],  "Kd": p["kd_roll"]},
        }
        actuator_params = p.get("actuator_params") or DEFAULT_ACTUATOR_PARAMS
        self.controller = HydrofoilController(pid_gains, dt=p["dt"], actuator_params=actuator_params)
        self._dw_t, self._dw_eps, self._eps_used = [], [], 0.0   # wing->tail downwash history (transport lag)
        self._vent = {"left": 0.0, "right": 0.0, "elev": 0.0,
                      "main_strut": 0.0, "main_strut_l": 0.0, "main_strut_r": 0.0, "elev_strut": 0.0}
        self._thrust_actual = 0.0                                 # lagged propulsion thrust [N]
        self.surge = SurgeController(Kp=p["kp_speed"], Ki=p["ki_speed"], Kd=p["kd_speed"],
                                      dt=p["dt"], max_thrust=p["max_thrust"])

        # Surface drives. "servo" (default): three PHYSICAL drives (left flap, right flap, elevator) after the
        # mixer, each a moteus position loop + BLDC + gearbox against the hinge moment (actuation/servo.py,
        # docs/17). "kinematic": the legacy delay -> rate limit -> lag on the three virtual channels.
        self.actuator_model = p.get("actuator_model", "servo")
        self.hinges = surface_models(p)
        self.lqr = None
        self.lqr_error: str | None = None                 # why there is no LQR (cannot trim at its design speeds)
        self._lqr_spare = None
        self.sensors = None
        self.estimator = None
        self.estimator_error: str | None = None
        self._last_cmds = dict.fromkeys(("delta_main", "delta_elev", "delta_roll"), 0.0)
        if p.get("sensors", "ideal") == "noisy":
            self._build_estimation()
        if p.get("controller", "pid") in ("lqr", "auto"):
            if self.actuator_model != "servo":
                import warnings
                warnings.warn(f"controller={p.get('controller')!r} needs actuator_model='servo'; falling back to the PID loops")
            else:
                self._build_lqr()
        sp = dict(DEFAULT_SERVO_PARAMS, **(p.get("servo_params") or {}))
        self.drives = {}
        self._last_target = None
        self._slew_last = None                              # command slew limiter state (cmd_slew_deg_s)
        for name, stop in (("left", p["flap_stop_deg"]), ("right", p["flap_stop_deg"]), ("elev", p["elev_stop_deg"])):
            spec = DriveSpec()
            spec.gear.ratio = p["gear_ratio"]
            spec.ctrl.v_bus = p["v_bus"]
            if p.get("supply_i_max") is not None:
                spec.ctrl.supply_i_max = p["supply_i_max"]
            if p.get("drive_i_cont") is not None:
                spec.ctrl.i_cont = p["drive_i_cont"]
            if p.get("shaft_diameter") is not None:
                spec.shaft_diameter = p["shaft_diameter"]
            if p.get("gear_rated_torque") is not None:
                spec.gear.rated_torque = p["gear_rated_torque"]
            self.drives[name] = ServoDrive(self.hinges[name], spec, p["dt"], stop, **sp,
                                           twist_k=self.twist_stiffness(name))
        # Control allocator (independent front flaps + elevator, Physics x Control-Stability bounds)
        self.allocator = ControlAllocator(p)
        self._last_alloc_status: dict = {}
        self._last_turn_envelope: dict = {}
        self._last_sigma: float = 99.0
        # Flight-mode supervisor (control/supervisor.py): off unless params["supervisor"] is True or a config dict.
        self.supervisor = None
        if p.get("supervisor"):
            from hydrofoil.control.supervisor import Supervisor
            self.supervisor = Supervisor(self, p["supervisor"] if isinstance(p["supervisor"], dict) else None)

    @property
    def hull_offset(self) -> float:
        """Effective hull-bottom vertical offset relative to the current CG [m] (hull_offset_nom - cg_z)."""
        return float(self.hull.hull_offset) if getattr(self, "hull", None) is not None else self._hull_offset_nominal - self._cg_z

    def set_cg_offset(self, offset, cg_z: float | None = None):
        """Move the centre of gravity by (dx forward, dy to starboard [, dz up]) [m] relative to the nominal
        layout, e.g. a rider shifting or standing. The body frame stays at the CG, so every foil, strut, and hull
        hydrostatic cell shifts by -offset in body NED coords. Can be called mid-flight."""
        dx, dy = float(offset[0]), float(offset[1])
        dz_up = float(offset[2]) if len(offset) >= 3 else (self._cg_z if cg_z is None else float(cg_z))
        self._cg_z = dz_up
        d = np.array([dx, dy, -dz_up])
        for n, f in self._foils().items():
            f.r_attach = self._r_nominal[n] - d
        if getattr(self, "hull", None) is not None:
            self.hull.set_cg_offset(dx, dy, -dz_up)
        self.params["cg_offset"] = (dx, dy)
        self.params["cg_z"] = dz_up
        self._eta_turn_cache = None

    def turning_efficiency(self, u: float) -> float:
        """
        Plant-derived steady-turn yaw efficiency eta_turn = r_turn * u / G from the linearized
        lateral-directional dynamics (`steady_turn_sensitivity` at trim). Lazily cached per geometry/CG.
        """
        p = self.params
        key = (
            round(self.hull_offset, 4),
            round(float(p["mass"]), 3),
            round(float(p["main_chord"]), 4),
            round(float(p["main_span"]), 4),
            round(float(p["main_strut_chord"]), 4),
            round(float(p["elev_strut_chord"]), 4),
            round(float(self.left_foil.r_attach[0]), 4),
            round(float(self.left_foil.r_attach[2]), 4),
        )
        if self._eta_turn_cache is None or self._eta_turn_cache[0] != key:
            etas: list[tuple[float, float]] = []
            if self.lqr is not None and abs(self.hull_offset - self._hull_offset_nominal) < 1e-4:
                for spd, tnu in zip(self.lqr.speeds, self.lqr.turn_nu):
                    etas.append((float(spd), float(np.clip(float(tnu[4]) * float(spd) / G, 0.05, 1.0))))
            if not etas:
                from hydrofoil.control.lqr import steady_turn_sensitivity
                from hydrofoil.linearize import linearize, trim
                eps_saved = self._eps_used
                try:
                    v_tgt = max(float(p.get("target_speed", 4.0)), 2.5)
                    v_grid = sorted({round(v, 2) for v in (0.65 * v_tgt, 0.85 * v_tgt, v_tgt, 1.2 * v_tgt) if v >= 2.0})
                    for spd in v_grid:
                        try:
                            xt, ut = trim(self, spd, u0=(5.0, 0.0, 0.0, 200.0), max_iter=50)
                            A, B, _, _ = linearize(self, xt, ut)
                            idx = [2, 3, 4, 7, 8, 9, 10, 11]
                            As, Bs = A[np.ix_(idx, idx)], B[np.ix_(idx, [0, 1, 2])]
                            tnu, _, _, _ = steady_turn_sensitivity(As, Bs)
                            etas.append((spd, float(np.clip(float(tnu[4]) * spd / G, 0.05, 1.0))))
                        except Exception:
                            continue
                finally:
                    self._eps_used = eps_saved
            if not etas:
                etas = [(max(float(p.get("target_speed", 4.0)), 2.0), 0.25)]
            spds = np.array([e[0] for e in etas], dtype=float)
            vals = np.array([e[1] for e in etas], dtype=float)
            self._eta_turn_cache = (key, spds, vals)
        _, spds, vals = self._eta_turn_cache
        return float(np.interp(max(float(u), 0.5), spds, vals))

    def _build_lqr(self):
        """Scheduled LQR (control/lqr.py), designed on a PID-configured copy of this plant."""
        self.lqr, self.lqr_error = self.make_lqr()
        if self.lqr is not None:
            self.lqr.auto_handback = (self.params.get("controller") == "auto")

    def make_lqr(self, params: dict | None = None):
        """Design a scheduled LQR for `params` (default: this boat's). Returns (controller, None), or (None, reason)
        when the boat cannot trim at enough of the scheduled speeds (e.g. a 0 deg wing below its flying speed), in
        which case it flies on the PID loops. Safe to call from a background thread."""
        from hydrofoil.control.lqr import LQRController
        # designed on the NOMINAL CG: the controller cannot know where the rider sits (cg_offset is the truth)
        p = dict(params if params is not None else self.params, controller="pid", cg_offset=(0.0, 0.0), cg_z=0.0,
                 supervisor=False)
        # design latency: the loop delay the gains are computed for. Defaults to the drive model's command
        # delay; on hardware set lqr_delay_s to the MEASURED sensor-to-surface latency.
        delay = p.get("lqr_delay_s")
        if delay is None:
            delay = dict(DEFAULT_SERVO_PARAMS, **(p.get("servo_params") or {}))["delay_s"]
        try:
            return LQRController(VehiclePhysics(p), p["dt"], weights=p.get("lqr_weights"), delay_s=delay,
                                 speeds=tuple(p.get("lqr_speeds", (3.0, 4.0, 5.0, 6.0, 7.0))),
                                 engage_speed=p.get("lqr_engage_speed", 3.2)), None
        except (RuntimeError, ValueError, np.linalg.LinAlgError) as e:
            import warnings
            warnings.warn(f"LQR not available for this boat ({e}); using the PID loops")
            return None, str(e)

    def set_main_incidence(self, deg: float) -> float:
        """Change the main wing's fixed mounting angle [deg] live. Physics follows at once; analytically shifts
        trim_ff_deg while the background rebuild re-derives LQR/Kalman schedules (see the viewer)."""
        deg = float(deg)
        old_deg = float(self.params.get("main_incidence_deg", 0.0))
        self.params["main_incidence_deg"] = deg
        for f in (self.left_foil, self.right_foil):
            f.geometry.aoa_offset = deg
        if abs(deg - old_deg) > 1e-9 and float(self.params.get("trim_ff_speed", 0.0)) > 0.0 and self.main_flap_eff > 1e-4:
            self.params["trim_ff_deg"] = float(self.params.get("trim_ff_deg", 0.0)) - (deg - old_deg) / self.main_flap_eff
        self._eta_turn_cache = None
        return deg

    def set_controller_mode(self, mode: str) -> str:
        """Switch flight controller mode at runtime between 'auto', 'lqr', and 'pid'."""
        m = str(mode).lower()
        if m not in ("auto", "lqr", "pid"):
            raise ValueError(f"unknown controller mode {mode!r}; expected 'auto', 'lqr', or 'pid'")
        self.params["controller"] = m
        if m == "pid":
            if self.lqr is not None:
                self.lqr.auto_handback = False
                self.lqr.force = False
        elif self.actuator_model == "servo":
            if self.lqr is None:
                if self._lqr_spare is not None:
                    self.lqr, self._lqr_spare = self._lqr_spare, None
                    self.lqr.reset()
                else:
                    self._build_lqr()
            if self.lqr is not None:
                self.lqr.auto_handback = (m == "auto")
                self.lqr.force = None
        return m

    def set_safety_margin(self, level: str) -> str:
        """Set live Layer-3 safety margin preset: 'conservative', 'standard', or 'sport'."""
        canon, _ = safety_preset_config(level)
        self.params["safety_margin"] = canon
        return canon

    def set_allocator_mode(self, mode: str) -> str:
        """Set live surface allocator mode: 'jacobian' or 'legacy'."""
        m = str(mode or "jacobian").strip().lower()
        if m not in ("jacobian", "legacy"):
            raise ValueError(f"unknown allocator mode {mode!r}; expected 'jacobian' or 'legacy'")
        self.params["allocator"] = m
        return m

    def controller_status(self, u: float = 0.0) -> dict:
        """Structured controller mode, handover state, blend fraction, and layman explanation."""
        mode = str(self.params.get("controller", "pid")).lower()
        if self.actuator_model != "servo":
            return {"mode": "pid", "active": "PID (kinematic)", "engaged": False,
                    "blending_in": False, "blending_out": False, "blend": 0.0,
                    "reason": "Legacy kinematic actuators active — running three SISO PID loops"}
        if mode == "pid" and self.lqr is None:
            return {"mode": "pid", "active": "PID", "engaged": False,
                    "blending_in": False, "blending_out": False, "blend": 0.0,
                    "reason": "Manual PID mode — three independent SISO loops control height, pitch, and roll"}
        if self.lqr is None:
            reason = ("PID active while LQR initializes" if not getattr(self, "lqr_error", None) else
                      "LQR unavailable: this boat cannot fly level at the LQR design speeds "
                      "(raise the wing mounting angle or the speed); the PID loops are flying")
            return {"mode": mode, "active": "PID", "engaged": False,
                    "blending_in": False, "blending_out": False, "blend": 0.0, "reason": reason}
        st = self.lqr.status_dict(u)
        st["mode"] = mode
        if self.supervisor is not None and self.supervisor.mode == "DISPLACEMENT":
            st["reason"] = f"Hull in water ({self.supervisor.mode}) — PID stabilizes roll/pitch while flaps stay neutral"
        return st

    def make_estimator(self, params: dict | None = None):
        """Design a scheduled KalmanEstimator on nominal CG. Returns (estimator, None) or (None, reason)."""
        from hydrofoil.estimation.kalman import KalmanEstimator
        p = dict(params if params is not None else self.params, controller="pid", sensors="ideal",
                 cg_offset=(0.0, 0.0), cg_z=0.0, supervisor=False)
        try:
            return KalmanEstimator(VehiclePhysics(p), p["dt"], process=p.get("kf_process"),
                                   speeds=tuple(p.get("lqr_speeds", (3.0, 4.0, 5.0, 6.0, 7.0)))), None
        except (RuntimeError, ValueError, np.linalg.LinAlgError) as e:
            return None, str(e)

    def _build_estimation(self):
        """Noisy sensors + Kalman estimator (params['sensors'] = 'noisy'); controllers then see estimates."""
        from hydrofoil.sensors.models import SensorSuite
        from hydrofoil.estimation.kalman import HeaveFilter
        p = self.params
        self.sensors = SensorSuite(seed=p.get("sensor_seed", 0), scale=p.get("sensor_noise_scale", 1.0),
                                   config=p.get("sensor_config"))
        self.estimator, self.estimator_error = self.make_estimator(p)
        self.heave = HeaveFilter(p["dt"], wave=p.get("kf_wave"), **(p.get("kf_heave") or {}))
        self._est_ready = False
        self._u_meas = 0.0
        self._init_height_monitor()

    def _init_height_monitor(self):
        p = self.params
        self._h_chans = (("height", "z"), ("height_aux", "z_aux")) if p.get("dual_height", True) else (("height", "z"),)
        self._h_vote = {
            k: {"stale_t": 0.0, "same_t": 0.0, "last_val": None, "bad": 0, "failed": False}
            for k, _ in self._h_chans
        }
        self.height_status = {
            "active": [k for k, _ in self._h_chans],
            "failed": [],
            "last_h": None,
            "last_fresh": False,
            "using_gnss": False,
        }

    def _control_state(self, state, t: float, surface_elev: float = 0.0) -> np.ndarray:
        """The state the controllers see: truth when sensors are 'ideal'; otherwise measurements (below the
        foilborne speed) or the Kalman estimate (above it). Physics always uses the true state."""
        if self.sensors is None:
            return state
        p = self.params
        dt = p["dt"]
        meas = self.sensors.measure(state, t, dt, surface_elev)
        if "u" in meas:
            self._u_meas = float(meas["u"])
        # Dual redundant height channels voted against the accelerometer-propagated HeaveFilter,
        # with automatic GNSS vertical-velocity fallback if all ranging sensors fail.
        if not self.heave.ready:
            for ch_name, mk in self._h_chans:
                if mk in meas:
                    self.heave.init(meas[mk])
                    self._h_vote[ch_name]["last_val"] = float(meas[mk])
                    break
        if self.heave.ready:
            self.heave.predict(meas.get("az"))
            timeout = self.supervisor.c["sensor_timeout"] if self.supervisor is not None else p.get("sensor_timeout", 0.5)
            check_freeze = self.supervisor is None or self.supervisor.mode in ("TAKEOFF", "FOILING", "LANDING")
            fresh_cands = []
            for ch_name, mk in self._h_chans:
                st = self._h_vote[ch_name]
                if st["failed"]:
                    continue
                is_fresh = mk in self.sensors.fresh and mk in meas
                if is_fresh:
                    hv = float(meas[mk])
                    st["stale_t"] = 0.0
                    if st["last_val"] is None or abs(hv - st["last_val"]) >= 1e-6:
                        st["same_t"] = 0.0
                    st["last_val"] = hv
                elif check_freeze:
                    st["stale_t"] += dt
                if check_freeze and st["last_val"] is not None:
                    st["same_t"] += dt
                    if st["same_t"] > timeout and (abs(self.heave.vz) > 0.05 or abs(st["last_val"] - self.heave.z) > 0.05):
                        st["failed"] = True
                        continue
                if check_freeze and st["stale_t"] > timeout:
                    st["failed"] = True
                    continue
                if is_fresh:
                    # Quarantine un-moving samples once boat is moving/diverging so they cannot poison HeaveFilter b_a
                    if st["same_t"] > 0.15 and (abs(self.heave.vz) > 0.05 or abs(hv - self.heave.z) > 0.03):
                        continue
                    sig = max(float(self.sensors.channels[ch_name]._sig[0]), 1e-4)
                    innov2 = self.heave.innov_z2(hv, sig * sig)
                    fresh_cands.append((ch_name, hv, sig, innov2))
            gate2 = self.heave.gate ** 2
            passed = [item for item in fresh_cands if item[3] < gate2]
            active_now = [k for k, _ in self._h_chans if not self._h_vote[k]["failed"]]
            for ch_name, hv, sig, innov2 in fresh_cands:
                st = self._h_vote[ch_name]
                if innov2 < gate2:
                    st["bad"] = 0
                elif passed or len(active_now) > 1:
                    st["bad"] += 1
                    if st["bad"] >= self.heave.max_reject:
                        st["failed"] = True
            h_use, r_use = None, None
            if passed:
                if len(passed) == 1:
                    h_use, r_use = passed[0][1], passed[0][2] ** 2
                else:
                    ws = [1.0 / (it[2] ** 2) for it in passed]
                    wsum = sum(ws)
                    h_use = sum(w * it[1] for w, it in zip(ws, passed)) / wsum
                    r_use = 1.0 / wsum
                self.heave.update_z(h_use, True, R_override=r_use)
            elif fresh_cands and len(active_now) <= 1:
                best = min(fresh_cands, key=lambda it: it[3])
                self.heave.update_z(best[1], True, R_override=best[2] ** 2)
                h_use = None
            active = [k for k, _ in self._h_chans if not self._h_vote[k]["failed"]]
            failed = [k for k, _ in self._h_chans if self._h_vote[k]["failed"]]
            using_gnss = bool(not active and p.get("gnss_fallback", True) and "vz_gnss" in meas)
            if using_gnss and "vz_gnss" in self.sensors.fresh:
                gch = self.sensors.channels["gnss_vz"]
                self.heave.update_vz(meas["vz_gnss"], sigma_vz=max(float(gch._sig[0]), 0.02),
                                     az=meas.get("az"), latency=float(gch.latency_s))
            self.height_status = {"active": active, "failed": failed, "last_h": h_use, "last_fresh": h_use is not None,
                                  "using_gnss": using_gnss}
            meas = dict(meas, z=self.heave.z)
        if not all(k in meas for k in ("z", "phi", "theta", "p", "q", "r")) and not self._est_ready:
            return state                                   # sensors not all up yet (first few ms)
        c = state.copy()
        z_trim = self.hull_offset - p["flying_height"]
        if self.estimator is None or (self._u_meas < p.get("est_engage_speed", 3.0) and not self._est_ready):
            for k, i in (("z", 2), ("phi", 3), ("theta", 4), ("p", 9), ("q", 10), ("r", 11)):
                if k in meas:
                    c[i] = meas[k]
            c[6] = self._u_meas
            c[7] = 0.0
            cos_tilt = max(float(np.cos(c[4]) * np.cos(c[3])), 0.15)
            c[8] = (self.heave.vz + self._u_meas * np.sin(c[4])) / cos_tilt if self.heave.ready else 0.0
            return c
        if not self._est_ready:
            self.estimator.init(meas, z_trim)
            self._est_ready = True
        last = self._last_cmds
        est = self.estimator.update(meas, [last["delta_main"], last["delta_elev"], last["delta_roll"]],
                                    self._u_meas, z_trim)
        c[[2, 3, 4, 7, 8, 9, 10, 11]] = est
        c[6] = self._u_meas
        # heave velocity from the kinematic filter (it sees wave and payload forces through the accelerometer,
        # which the model-based filter cannot): vz = -u sin(theta) + v cos(theta) sin(phi) + w cos(theta) cos(phi)
        c[2] = self.heave.z
        cos_tilt = max(float(np.cos(c[4]) * np.cos(c[3])), 0.15)
        c[8] = (self.heave.vz + self._u_meas * np.sin(c[4]) - c[7] * np.cos(c[4]) * np.sin(c[3])) / cos_tilt
        return c

    def reset(self):
        """Reset controllers and drive states (e.g. on a sim restart)."""
        if getattr(self, "sensors", None) is not None:
            self.sensors.reset()
            if getattr(self, "estimator", None) is not None:
                self.estimator.reset()
            self.heave.reset()
            self._est_ready = False
            self._last_cmds = dict.fromkeys(("delta_main", "delta_elev", "delta_roll"), 0.0)
            self._init_height_monitor()
        if getattr(self, "lqr", None) is not None:
            self.lqr.reset()
            if self.params.get("controller") == "pid":
                self._lqr_spare, self.lqr = self.lqr, None
        self._dw_t, self._dw_eps, self._eps_used = [], [], 0.0
        self._vent = {"left": 0.0, "right": 0.0, "elev": 0.0,
                      "main_strut": 0.0, "main_strut_l": 0.0, "main_strut_r": 0.0, "elev_strut": 0.0}
        self._thrust_actual = 0.0
        self.controller.reset()
        self.surge.reset()
        if getattr(self, "supervisor", None) is not None:
            self.supervisor.reset()
        for d in self.drives.values():
            d.reset()
        self._slew_last = None
        self._last_alloc_status = {}
        self._last_turn_envelope = {}
        self._last_sigma = 99.0

    def twist_stiffness(self, name: str) -> float | None:
        """Spanwise twist stiffness of a surface at its drive [N m/rad], for the span-mean deflection under a
        uniform hinge load: 3 GJ / L for a flap driven at one end (L = flapped span); the all-moving elevator is
        driven at mid-span through its strut, two halves of L/2 in parallel: 12 GJ / L. None = rigid."""
        p = self.params
        if name in ("left", "right"):
            gj = p.get("main_flap_GJ")
            L = p["main_span"] * (p.get("main_flap_span_frac", 1.0) if p.get("main_actuation", "flap") == "flap" else 1.0)
            return None if (gj is None or L <= 1e-4) else 3.0 * gj / L
        gj = p.get("elev_GJ")
        return None if (gj is None or p["elev_span"] <= 1e-4) else 12.0 * gj / p["elev_span"]

    def _compute_sigma_min(self, state, env) -> float:
        """Compute minimum cavitation number across submerged foils."""
        eta, nu = state[:6], state[6:]
        sigma_min = 99.0
        for name, foil in (("left", self.left_foil), ("right", self.right_foil), ("elev", self.elev_foil)):
            vb = env.foil_vel_body[name]
            speed = math.sqrt((float(nu[0]) - float(vb[0])) ** 2 + (float(nu[1]) - float(vb[1])) ** 2 + (float(nu[2]) - float(vb[2])) ** 2)
            hc = self._effective_submergence_over_chord(foil, eta, env.foil_surface_z[name])
            if hc > 0.0 and speed >= 2.0:
                sigma_min = min(sigma_min, cavitation_number(speed, hc * foil.geometry.chord, rho=foil.rho))
        return float(sigma_min)

    def _servo_commands(self, raw: dict, state, env, u_meas: float | None = None, alloc_state=None) -> dict:
        """Allocate virtual controller channels onto the physical drives and advance drive dynamics."""
        p = self.params
        u = max(float(state[6] if u_meas is None else u_meas), 0.1)
        vs = p.get("q_schedule_speed", 0.0)
        k = float(np.clip((vs / u) ** 2, 0.25, 1.0)) if vs > 0 else 1.0   # gain-schedule flap commands by 1/q above vs
        raw_s = {"delta_main": k * raw["delta_main"], "delta_roll": k * raw["delta_roll"], "delta_elev": k * raw["delta_elev"]}
        flows_true = flow_models(self, state, env, self.hinges)
        self._last_flows = flows_true                     # hinge model at this flow (supervisor fault residuals)
        if self._slew_last is None:
            self._slew_last = {n: float(np.degrees(self.drives[n].delta)) for n in self.drives}

        jam = self.supervisor.jam if self.supervisor is not None else None
        use_truth = self.sensors is None
        s_alloc = state if (use_truth or alloc_state is None) else alloc_state
        env_alloc = env if use_truth else self.calm_sample()
        if p.get("allocator", "jacobian") == "jacobian" and self.allocator is not None:
            flows_alloc = flows_true if use_truth else flow_models(self, s_alloc, env_alloc, self.hinges, r_overrides=self._r_nominal)
            target, alloc_status = self.allocator.allocate(
                self, raw_s, s_alloc, env_alloc, self._slew_last, u_meas=u_meas,
                jam=jam, flows=flows_alloc, use_truth=use_truth,
            )
            self._slew_last = dict(target)
            self._last_alloc_status = alloc_status
            self._last_sigma = float(alloc_status.get("sigma_min", 99.0))
        else:
            main, roll, elev = raw_s["delta_main"], raw_s["delta_roll"], raw_s["delta_elev"]
            target = {"left": main - roll, "right": main + roll, "elev": elev}
            if self.supervisor is not None:
                target = self.supervisor.mix(target)          # jammed-flap reconfiguration
            slew = p.get("cmd_slew_deg_s")
            if slew:                                           # rate-limit each drive's command (takeoff steps)
                step = slew * p["dt"]
                target = {n: float(np.clip(v, self._slew_last[n] - step, self._slew_last[n] + step))
                          for n, v in target.items()}
                self._slew_last = dict(target)
            self._last_sigma = self._compute_sigma_min(s_alloc, env_alloc)
            fl_s, fr_s, fe_s = float(p["flap_stop_deg"]), float(p["flap_stop_deg"]), float(p["elev_stop_deg"])
            m_ach = 0.5 * (target["left"] + target["right"])
            r_ach = 0.5 * (target["right"] - target["left"])
            self._last_alloc_status = {
                "active_mode": "legacy",
                "safety_margin": str(p.get("safety_margin", "standard")),
                "angles": dict(target),
                "bounds": {"left": [-fl_s, fl_s], "right": [-fl_s, fl_s], "elev": [-fe_s, fe_s]},
                "binding": {"left": "legacy mixer", "right": "legacy mixer", "elev": "legacy mixer"},
                "split_deg": float(target["right"] - target["left"]),
                "reverse_drive_enabled": bool(p.get("reverse_drive_enabled", True)),
                "reverse_drive_active": bool(target["left"] * target["right"] < -0.25 and abs(target["right"] - target["left"]) > 1.0),
                "reverse_drive_gated": False,
                "reverse_drive_reason": "Legacy mixer",
                "lift_margin": 1.0,
                "elev_reserve": float(np.clip(1.0 - abs(target["elev"]) / max(fe_s, 1.0), 0.0, 1.0)),
                "sigma_min": float(self._last_sigma),
                "saturated": {
                    "main": abs(m_ach - raw_s["delta_main"]) > 0.15 or (abs(target["left"]) >= fl_s - 0.05 and abs(target["right"]) >= fl_s - 0.05),
                    "elev": abs(target["elev"] - raw_s["delta_elev"]) > 0.15 or abs(target["elev"]) >= fe_s - 0.05,
                    "roll": abs(r_ach - raw_s["delta_roll"]) > 0.15,
                },
                "solve_ms": 0.0,
            }
        self._last_target = target
        for d in self.drives.values():                     # live gear ratio / bus voltage (UI sliders)
            d.spec.gear.ratio, d.spec.ctrl.v_bus = p["gear_ratio"], p["v_bus"]
        ang = {n: self.drives[n].update(target[n], flows_true[n]) for n in self.drives}
        return {"delta_main": 0.5 * (ang["left"] + ang["right"]),
                "delta_roll": 0.5 * (ang["right"] - ang["left"]),
                "delta_left": float(ang["left"]),
                "delta_right": float(ang["right"]),
                "delta_elev": float(ang["elev"]),
                "target": target,
                "alloc": dict(self._last_alloc_status),
                "drive": {n: dict(self.drives[n].telemetry) for n in self.drives}}

    def _downwash_qs(self) -> float:
        """Quasi-steady downwash angle at the tail [rad] from the main foils' latest corrected CL:
        eps = k * 2 CL / (pi AR_wing), AR on the full two-panel span."""
        l, r = self.left_foil, self.right_foil
        ar_w = (l.geometry.span + r.geometry.span) / max(l.geometry.chord, 1e-4)
        if ar_w <= 1e-4:
            return 0.0
        return self.params.get("downwash_factor", 1.0) * (l.last_CL + r.last_CL) / (np.pi * ar_w)

    def _downwash_lagged(self, t: float, u: float) -> float:
        """Downwash the tail sees NOW: the wing's value from l/u seconds ago (the wake must be convected
        the wing->tail distance). params['downwash_lag'] = False uses the instantaneous value."""
        if not self.params.get("downwash_lag", True) or not self._dw_t:
            return self._downwash_qs() if not self.params.get("downwash_lag", True) else 0.0
        l = abs(self.elev_foil.r_attach[0] - self.left_foil.r_attach[0])
        return float(np.interp(t - l / max(float(u), 0.5), self._dw_t, self._dw_eps))

    def _trim_ff(self, u: float) -> float:
        """Speed-scheduled trim flap angle [deg], added to the heave PID output so the integrator no longer
        has to ramp up to the weight-supporting angle. Lift ~ u^2, so the angle of attack the wing needs goes
        as 1/u^2; the fixed incidence supplies part of it and the flap the rest:
            eff * delta_trim(u) + incidence = (eff * d0 + incidence) * (vs / u)^2."""
        p = self.params
        d0, vs = p.get("trim_ff_deg", 0.0), p.get("trim_ff_speed", 0.0)
        if (abs(d0) < 1e-9 and abs(float(p.get("main_incidence_deg", 0.0))) < 1e-9) or vs <= 0.0 or self.main_flap_eff <= 1e-6:
            return 0.0
        inc = p.get("main_incidence_deg", 0.0) / self.main_flap_eff    # incidence in flap degrees
        cmd_lim = float(p.get("flap_cmd_max_deg", min(20.0, float(p.get("flap_stop_deg", 30.0)))))
        return float(np.clip((d0 + inc) * (vs / max(float(u), 0.5 * vs)) ** 2 - inc, -cmd_lim, cmd_lim))

    def rest_state(self) -> np.ndarray:
        """Initial state: at rest, floating at the hull's buoyant equilibrium draft across hull and any boxes."""
        p = self.params
        h = self.hull
        m_need = float(p["mass"]) / RHO_WATER
        if not h.boxes:
            rest_draft = min(m_need / max(h.waterplane_area, 1e-6), h.max_draft)
        else:
            a_main = h.n_hulls * float(p["hull_length"]) * float(p["hull_beam"])
            cuts = sorted({0.0, float(h.max_draft), *(float(b["height"]) for b in h.boxes)})
            rest_draft = cuts[-1]
            for lo_c, hi_c in zip(cuts[:-1], cuts[1:]):
                v_lo = min(lo_c, h.max_draft) * a_main + sum(min(lo_c, b["height"]) * b["length"] * b["beam"] for b in h.boxes)
                v_hi = min(hi_c, h.max_draft) * a_main + sum(min(hi_c, b["height"]) * b["length"] * b["beam"] for b in h.boxes)
                if v_hi >= m_need - 1e-9 and v_hi > v_lo + 1e-9:
                    rest_draft = lo_c + (m_need - v_lo) / (v_hi - v_lo) * (hi_c - lo_c)
                    break
        z0 = self.hull_offset + rest_draft
        eta0 = np.array([0.0, 0.0, z0, 0.0, np.radians(2.0), 0.0])
        nu0 = np.zeros(6)
        return np.concatenate([eta0, nu0])

    def _foils(self):
        out = {"left": self.left_foil, "right": self.right_foil, "elev": self.elev_foil,
               "elev_strut": self.elev_strut}
        if self._dual_front_struts:
            out["main_strut_l"] = self.main_strut_l
            out["main_strut_r"] = self.main_strut_r
        else:
            out["main_strut"] = self.main_strut
        return out

    def wave_sample(self, eta, t, wave) -> WaveSample:
        """Sample the wave field at the hull and at every foil's world position."""
        R = R_body_to_ned(eta[3], eta[4], eta[5])
        smp = WaveSample()
        smp.elevation = wave.elevation(eta[0], eta[1], t)
        smp.hull_surface_z = -smp.elevation          # NED: surface risen -> smaller (more negative) z
        cells = self.hull.cell_world_xy(eta)         # where each hull footprint cell actually is in the world
        elev_cells = np.zeros(self.hull.n) + wave.elevation(cells[:, 0], cells[:, 1], t)
        smp.hull_cell_surface_z = -elev_cells
        for name, foil in self._foils().items():
            pos = eta[:3] + R @ foil.r_attach        # foil AC in the NED world frame
            elev = wave.elevation(pos[0], pos[1], t)
            smp.foil_surface_z[name] = -elev
            # orbital_velocity wants depth below the still surface as a NEGATIVE z (z <= 0)
            v_ned = wave.orbital_velocity(pos[0], pos[1], -pos[2], t)
            smp.foil_vel_body[name] = R.T @ v_ned
        if self._dual_front_struts:
            smp.foil_surface_z["main_strut"] = 0.5 * (smp.foil_surface_z["main_strut_l"] + smp.foil_surface_z["main_strut_r"])
            smp.foil_vel_body["main_strut"] = 0.5 * (smp.foil_vel_body["main_strut_l"] + smp.foil_vel_body["main_strut_r"])
        return smp

    @staticmethod
    def calm_sample() -> WaveSample:
        z3 = np.zeros(3)
        names = ("left", "right", "elev", "main_strut", "main_strut_l", "main_strut_r", "elev_strut")
        return WaveSample(0.0, 0.0, 0.0, {n: 0.0 for n in names}, {n: z3 for n in names})

    def _submergence_over_chord(self, foil, eta, water_surface_z, r_attach: np.ndarray | None = None):
        """
        This foil's depth below the (possibly wave-displaced) local water
        surface at its attachment centre r_attach, divided by its chord.
        """
        phi, th = float(eta[3]), float(eta[4])
        ct = math.cos(th)
        r = foil.r_attach if r_attach is None else r_attach
        world_z = float(eta[2]) + (-math.sin(th)) * float(r[0]) + (ct * math.sin(phi)) * float(r[1]) + (ct * math.cos(phi)) * float(r[2])
        return (world_z - float(water_surface_z)) / foil.geometry.chord

    def _foil_tip_submergence_over_chord(self, foil, eta, water_surface_z, r_attach: np.ndarray | None = None) -> float:
        """
        Shallowest wingtip depth below local water surface, divided by chord.
        For split left/right panels (|r_attach[1]| > 1e-6), evaluates the outboard tip
        at y_tip = r_attach[1] + sign(y) * span/2; for centerline foils (elevator),
        takes the shallower of the two tips (+-span/2). Identical to
        `_submergence_over_chord` when level (phi == 0).
        """
        phi, th = float(eta[3]), float(eta[4])
        ct = math.cos(th)
        r2y = ct * math.sin(phi)
        r = foil.r_attach if r_attach is None else r_attach
        z_ctr = float(eta[2]) + (-math.sin(th)) * float(r[0]) + r2y * float(r[1]) + (ct * math.cos(phi)) * float(r[2])
        dy = r2y * (0.5 * foil.geometry.span)
        ry = float(r[1])
        if abs(ry) > 1e-6:
            z_tip = z_ctr + (dy if ry > 0.0 else -dy)
        else:
            z_tip = z_ctr - abs(dy)
        return (z_tip - float(water_surface_z)) / foil.geometry.chord

    def _effective_submergence_over_chord(self, foil, eta, water_surface_z, r_attach: np.ndarray | None = None) -> float:
        """
        Span-averaged effective submergence h/c for a horizontal foil using 3-station
        Simpson quadrature (inboard edge, center, outboard tip) of `free_surface_lift_factor`.
        Identical to `_submergence_over_chord` in level flight (phi == 0), while smoothly
        capturing outboard wingtip free-surface lift loss and emergence in banked turns.
        """
        phi, th = float(eta[3]), float(eta[4])
        ct = math.cos(th)
        r2y = ct * math.sin(phi)
        r = foil.r_attach if r_attach is None else r_attach
        c = foil.geometry.chord
        z_ctr = float(eta[2]) + (-math.sin(th)) * float(r[0]) + r2y * float(r[1]) + (ct * math.cos(phi)) * float(r[2])
        hc_ctr = (z_ctr - float(water_surface_z)) / c
        dy_hc = r2y * (0.5 * foil.geometry.span) / c
        if abs(dy_hc) < 1e-9:
            return hc_ctr
        hc_a = hc_ctr - dy_hc
        hc_b = hc_ctr + dy_hc
        if hc_a <= 0.0 and hc_ctr <= 0.0 and hc_b <= 0.0:
            return min(hc_a, hc_ctr, hc_b)
        def _fs(h):
            if h <= 0.0: return 0.0
            return 1.0 if h > 10.0 else (1.0 - math.exp(-h / 1.58))
        f_avg = (_fs(hc_a) + 4.0 * _fs(hc_ctr) + _fs(hc_b)) / 6.0
        if f_avg <= 1e-9:
            return 0.0
        if f_avg >= 1.0 - 1e-12:
            return hc_ctr
        return float(-1.58 * math.log(1.0 - f_avg))

    def physics_rhs(self, state, cmds, thrust, env: WaveSample | None = None):
        """
        Pure nonlinear dynamics d(state)/dt. env is a WaveSample (None = calm).
        Each foil sees the body velocity minus the wave orbital velocity AT ITS OWN
        position; hull drag and heave damping use the raw body velocity.
        """
        p = self.params
        env = env or self.calm_sample()
        eta, nu = state[:6], state[6:]

        def foil_force(name, delta, downwash=0.0):
            foil = self._foils()[name]
            nu_rel = nu.copy()
            nu_rel[:3] -= env.foil_vel_body[name]
            return foil.generalized_force(
                nu_rel, delta=delta,
                submergence_over_chord=self._effective_submergence_over_chord(foil, eta, env.foil_surface_z[name]),
                downwash_rad=downwash,
                ventilation=self._vent.get(name, 0.0))

        def strut_force(name):
            strut = self._foils()[name]
            nu_rel = nu.copy()
            nu_rel[:3] -= env.foil_vel_body[name]
            h = self._submergence_over_chord(strut, eta, env.foil_surface_z[name]) * strut.geometry.chord
            cos_tilt = max(float(np.cos(eta[3]) * np.cos(eta[4])), 0.25)
            strut_len = self.hull_offset + strut.r_attach[2]
            wet = float(np.clip(h / cos_tilt, 0.0, strut_len))
            r = strut.r_attach - np.array([0.0, 0.0, 0.5 * wet])   # AC at mid wetted length
            return strut.generalized_force(nu_rel, span=wet, r_attach=r, ventilation=self._vent.get(name, 0.0))

        tau = np.zeros(6)
        if self._dual_front_struts:
            n_front = max(int(p.get("main_strut_count", 2)), 2)
            scale_front = 0.5 * n_front
            tau += (strut_force("main_strut_l") + strut_force("main_strut_r")) * scale_front
        else:
            tau += strut_force("main_strut") * max(int(p.get("main_strut_count", 1)), 1)
        tau += strut_force("elev_strut")
        dl_raw = cmds["delta_left"] if "delta_left" in cmds else (cmds["delta_main"] - cmds["delta_roll"])
        dr_raw = cmds["delta_right"] if "delta_right" in cmds else (cmds["delta_main"] + cmds["delta_roll"])
        k_cc = float(np.clip(p.get("split_flap_cross_coupling", 0.0), 0.0, 0.35))
        if k_cc > 0.0 and dl_raw != dr_raw:
            dl_eff = (1.0 - k_cc) * dl_raw + k_cc * dr_raw
            dr_eff = (1.0 - k_cc) * dr_raw + k_cc * dl_raw
        else:
            dl_eff, dr_eff = dl_raw, dr_raw
        tau += foil_force("left", dl_eff)
        tau += foil_force("right", dr_eff)
        # Wing -> tail downwash: self._eps_used (transport-lagged, see _downwash_lagged)
        tau += foil_force("elev", cmds["delta_elev"], downwash=self._eps_used)
        tau += self.hull.buoyancy(eta, env.hull_cell_surface_z)
        tau += gravity_force(p["mass"], eta)
        tau += self.hull.box_drag(nu, eta, env.hull_cell_surface_z, rot_damping=0.35 * p["hull_rot_damping"])
        tau += self.hull.drag(nu, self.hull.mean_draft(eta, env.hull_cell_surface_z),
                              p["hull_drag_coeff"], p["hull_rot_damping"],
                              hull_length=p["hull_length"], hump_amp=p.get("hull_hump_amp", 0.8))
        tau[0] += thrust
        z_prop = float(p.get("prop_pos_z", 0.0)) + self._cg_z
        if z_prop != 0.0:
            tau[4] += thrust * z_prop
        tau[2] -= p["heave_damping"] * nu[2]
        return np.concatenate([J(eta) @ nu, self.rigid_body.nu_dot(nu, tau)])

    def step(self, state: np.ndarray, t: float, wave, cmds_override: dict | None = None):
        """
        Advance one full RK4 step: evaluate the controller and surge
        controller once (held fixed across the RK4 sub-stages, same as
        wave state), integrate, and return everything a caller needs to
        broadcast/log.

        cmds_override : optional dict of delta_main/delta_elev/delta_roll [deg] that
                        bypasses the controller (open-loop experiments).

        Returns
        -------
        new_state, cmds, thrust, draft, wave_elev   (draft is measured on new_state)
        """
        p = self.params
        dt = p["dt"]
        eta, nu = state[:6], state[6:]

        # phi_ref/theta_ref are stored in degrees (UI convention); eta's
        # angles are radians (Fossen convention) — convert before compare.
        env = self.wave_sample(eta, t, wave)
        cstate = self._control_state(state, t, env.elevation)   # what the controllers see (== state if sensors ideal)
        if self.supervisor is not None and cmds_override is None:
            self.supervisor.update(t, cstate)                  # mode, references, limits (control/supervisor.py)
        else:
            w_amp = float(wave.amplitude) if hasattr(wave, "amplitude") else float(p.get("wave_amplitude", 0.0))
            self._last_turn_envelope = compute_turn_envelope(
                self, cstate=cstate, wave_amp=w_amp, height_ref=p.get("flying_height"),
                alloc_status=self._last_alloc_status,
            )
        z_ref = self.hull_offset - p["flying_height"]
        ceta, cnu = cstate[:6], cstate[6:]
        self._eps_used = self._downwash_lagged(t, nu[0])      # frozen over the step; drive loads use it too
        if cmds_override is None and self.actuator_model == "servo":
            ff = self._trim_ff(cnu[0])
            cmd_lim = float(p.get("flap_cmd_max_deg", min(20.0, float(p.get("flap_stop_deg", 30.0)))))
            self.controller.heave_pid.output_min = -cmd_lim - ff   # anti-windup judges the TOTAL flap command
            self.controller.heave_pid.output_max = cmd_lim - ff
            pid_active = not (self.lqr is not None and self.lqr.engaged and self.lqr.force is not False
                              and self.lqr.since_engage >= self.lqr.blend_s)
            raw = self.controller.pid_commands(
                z_ref,                       ceta[2],
                np.radians(p["phi_ref"]),    ceta[3],
                np.radians(p["theta_ref"]),  ceta[4],
                integrate=pid_active,
            )
            raw["delta_main"] += ff
            if self.lqr is not None:
                self.lqr.hold_heave = self.supervisor is not None and self.supervisor.mode == "DISPLACEMENT"
                dl, dr, de = (self.drives[n] for n in ("left", "right", "elev"))   # drive encoders: angle, rate
                act = ([0.5 * np.degrees(dl.delta + dr.delta), np.degrees(de.delta), 0.5 * np.degrees(dr.delta - dl.delta)],
                       [0.5 * np.degrees(dl.rate + dr.rate), np.degrees(de.rate), 0.5 * np.degrees(dr.rate - dl.rate)])
                cur_draft = self.hull.wetted_draft(eta, env.hull_cell_surface_z)
                raw = self.lqr.update(cstate, z_ref, np.radians(p["phi_ref"]), np.radians(p["theta_ref"]),
                                      raw, act, draft=cur_draft)
            if self.supervisor is not None:
                raw = self.supervisor.shape_raw(raw, cstate)
            cmds = self._servo_commands(raw, state, env, u_meas=cnu[0], alloc_state=cstate)
            if self.lqr is not None:
                t_cmd = cmds["target"]
                sat_d = cmds["alloc"].get("saturated") or {}
                self.lqr.sync_sent(
                    [0.5 * (t_cmd["left"] + t_cmd["right"]), t_cmd["elev"], 0.5 * (t_cmd["right"] - t_cmd["left"])],
                    sat_flags=(bool(sat_d.get("main")), bool(sat_d.get("elev")), bool(sat_d.get("roll"))),
                )
        elif cmds_override is None:
            raw_k = self.controller.pid_commands(
                z_ref,                       ceta[2],
                np.radians(p["phi_ref"]),    ceta[3],
                np.radians(p["theta_ref"]),  ceta[4],
            )
            if self.supervisor is not None:
                raw_k = self.supervisor.shape_raw(raw_k, cstate)
            cmds = {
                "delta_main": self.controller.main_actuator.update(raw_k["delta_main"]),
                "delta_elev": self.controller.elev_actuator.update(raw_k["delta_elev"]),
                "delta_roll": self.controller.roll_actuator.update(raw_k["delta_roll"]),
            }
            self._last_sigma = self._compute_sigma_min(cstate, env if self.sensors is None else self.calm_sample())
        else:
            cmds = dict(cmds_override)
            self._last_sigma = self._compute_sigma_min(state, env)
        thrust_cmd = self.surge.update(p["target_speed"], cnu[0])
        p_prop = p.get("prop_power_max")
        if p_prop is not None and p_prop > 0.0 and self.surge.pid.output_max > 0.0:
            thrust_cmd = min(thrust_cmd, float(p_prop) / max(float(nu[0]), 1.0))
        prop_tau = p.get("prop_tau_s", 0.0)
        if prop_tau > 0.0 and cmds_override is None:
            alpha_p = 1.0 - float(np.exp(-dt / prop_tau))
            self._thrust_actual += alpha_p * (thrust_cmd - self._thrust_actual)
            thrust = self._thrust_actual
        else:
            thrust = self._thrust_actual = thrust_cmd
        self._last_cmds = {k: cmds[k] for k in ("delta_main", "delta_elev", "delta_roll")}
        wave_elev = env.elevation

        def rhs(s):
            return self.physics_rhs(s, cmds, thrust, env)

        k1 = rhs(state)
        k2 = rhs(state + dt / 2 * k1)
        k3 = rhs(state + dt / 2 * k2)
        k4 = rhs(state + dt * k3)
        new_state = state + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

        if not np.all(np.isfinite(new_state)):
            # Recovered, not ignored: snap back to the last valid state with
            # zero velocity/rates and reset the controllers, so the pilot
            # regains control instead of the app streaming NaN forever.
            new_state = state.copy()
            new_state[6:] = 0.0
            self.reset()
        else:
            new_state[3] = np.clip(new_state[3], -MAX_ATTITUDE_RAD, MAX_ATTITUDE_RAD)
            new_state[4] = np.clip(new_state[4], -MAX_ATTITUDE_RAD, MAX_ATTITUDE_RAD)
            new_state[6:9] = np.clip(new_state[6:9], -MAX_LINEAR_VEL, MAX_LINEAR_VEL)
            new_state[9:12] = np.clip(new_state[9:12], -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)

        # Update per-foil surface breach ventilation (wingtip-first) & strut sideslip ventilation
        if p.get("ventilation_model", True):
            u_new = float(new_state[6])
            hc_trig = p.get("vent_breach_hc", 0.25)
            hc_rec = p.get("vent_recover_hc", 0.85)
            for name, foil in (("left", self.left_foil), ("right", self.right_foil), ("elev", self.elev_foil)):
                hc_ctr = self._submergence_over_chord(foil, new_state[:6], env.foil_surface_z[name])
                hc_tip = self._foil_tip_submergence_over_chord(foil, new_state[:6], env.foil_surface_z[name])
                hc_min = min(hc_ctr, hc_tip)
                v = self._vent[name]
                if hc_min < hc_trig and u_new > 2.0:
                    # Partial tip-only breach ventilates outboard portion (~0.55); full panel breach -> 1.0
                    v_tgt = 1.0 if hc_ctr < hc_trig else float(np.clip(0.55 + 0.45 * (hc_trig - hc_tip) / max(hc_trig + 0.5, 0.5), 0.55, 1.0))
                    if v < v_tgt:
                        v += (v_tgt - v) * min(1.0, dt / 0.08)
                elif v > 0.0 and (hc_min > hc_rec or u_new < 1.5):
                    v -= v * min(1.0, dt / 0.40)
                self._vent[name] = 0.0 if v < 1e-4 else float(min(1.0, v))
            # Vertical strut sideslip ventilation (surface-piercing air entrainment at high |beta|)
            beta_trig = float(p.get("strut_vent_deg", 8.5))
            beta_rec = float(p.get("strut_vent_recover_deg", 4.5))
            cos_tilt = max(float(np.cos(new_state[3]) * np.cos(new_state[4])), 0.25)
            strut_items = (
                (("main_strut_l", self.main_strut_l), ("main_strut_r", self.main_strut_r), ("elev_strut", self.elev_strut))
                if self._dual_front_struts
                else (("main_strut", self.main_strut), ("elev_strut", self.elev_strut))
            )
            for name, strut in strut_items:
                h = self._submergence_over_chord(strut, new_state[:6], env.foil_surface_z[name]) * strut.geometry.chord
                wet = float(np.clip(h / cos_tilt, 0.0, self.hull_offset + strut.r_attach[2]))
                r_mid = strut.r_attach - np.array([0.0, 0.0, 0.5 * wet])
                nu_rel = new_state[6:].copy()
                nu_rel[:3] -= env.foil_vel_body[name]
                V_s, alpha_s = strut.flow(nu_rel, r_attach=r_mid)
                beta_s = abs(float(np.degrees(alpha_s)))
                v = self._vent[name]
                if wet > 0.1 and V_s > 3.5 and beta_s > beta_trig:
                    v += (1.0 - v) * min(1.0, dt / 0.08)
                elif v > 0.0 and (beta_s < beta_rec or V_s < 2.0 or wet <= 0.1):
                    v -= v * min(1.0, dt / 0.40)
                self._vent[name] = 0.0 if v < 1e-4 else float(min(1.0, v))
            if self._dual_front_struts:
                self._vent["main_strut"] = max(self._vent["main_strut_l"], self._vent["main_strut_r"])

        # last_CL is from the final RK stage (~ end of step): record it for the transport delay
        self._dw_t.append(t + dt); self._dw_eps.append(self._downwash_qs())
        while len(self._dw_t) > 1 and self._dw_t[1] < t + dt - 5.0:
            del self._dw_t[0], self._dw_eps[0]
        draft = self.hull.wetted_draft(new_state[:6], env.hull_cell_surface_z)
        return new_state, cmds, thrust, draft, wave_elev
