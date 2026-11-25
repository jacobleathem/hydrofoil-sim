"""
Speed-scheduled, delay-aware LQR flight controller (docs/10 Phase 3, item 2; docs/19).

One coupled MIMO law replaces the three SISO PIDs once the boat is foilborne:

    c = delta_trim(u) - K(u) [ z - z_ref, phi - phi_ref, theta - theta_ref, v, w, p, q, r,
                               int(z - z_ref), int(phi - phi_ref), int(theta - theta_ref),
                               delta - delta_trim, delta_dot, c[k-1] - delta_trim, ..., c[k-d] - delta_trim ]

on the three virtual flap channels (delta_main, delta_elev, delta_roll [deg]). The design model
(`augmented_model`) is the linearised plant (`hydrofoil.linearize`) driven by the ACHIEVED surface angles,
plus a second-order model of the drives and a chain of d = delay / dt past commands, discretised at the
control rate; the cost also penalises the command rate. Gains and trim are computed at a few speeds and
interpolated in the surge speed u (thrust stays on the surge PID). Integral action removes the steady
error that proportional state feedback leaves under a bank/pitch reference or a trim shift.

Why the drive and the delay are in the model: the earlier design (`design`, kept for reference and tests)
assumed the commanded flap was delivered instantly. Its roll and pitch loops crossed over near the drive's
own bandwidth, and it limit-cycled as soon as the loop latency reached 30 ms.

Takeoff and low speed are outside the linear model's validity, so the PID flies them; the handover is
bumpless on all three channels (the integrators are initialised so the first LQR command equals the last
PID command, within the integrator clamps) and latches.
"""

from __future__ import annotations

import numpy as np
from scipy.linalg import expm, solve_continuous_are, solve_discrete_are

from hydrofoil.constants import G
from hydrofoil.linearize import trim, linearize, reduced

STATE_IDX = [0, 1, 2, 4, 5, 6, 7, 8]          # of the reduced [z phi theta u v w p q r]: drop u (index 3)

# Bryson-style defaults: 1 / (acceptable excursion)^2. Angles in rad, rates in rad/s, inputs in degrees.
DEFAULT_WEIGHTS = {
    "z": 0.03, "phi": np.radians(0.8), "theta": np.radians(2.0),
    "v": 0.3, "w": 0.15, "p": np.radians(8.0), "q": np.radians(8.0), "r": np.radians(15.0),
    "iz": 0.03 * 1.0, "iphi": np.radians(0.8), "itheta": np.radians(2.0) * 1.0,
    "u_main": 8.0, "u_elev": 8.0, "u_roll": 8.0,
    "du_main": 250.0, "du_elev": 250.0, "du_roll": 250.0,   # command rates [deg/s] (delay-aware design only)
}


def _q_r(w: dict):
    q = np.diag([1 / w[k] ** 2 for k in ("z", "phi", "theta", "v", "w", "p", "q", "r", "iz", "iphi", "itheta")])
    r = np.diag([1 / w["u_main"] ** 2, 1 / w["u_elev"] ** 2, 1 / w["u_roll"] ** 2])
    return q, r


def design(vp, speed: float, weights: dict | None = None):
    """LQR gain K (3 x 11) and trim flap angles (3,) at one speed."""
    w = dict(DEFAULT_WEIGHTS, **(weights or {}))
    x0, u0 = trim(vp, speed)
    A, B = linearize(vp, x0, u0)
    Ar, Br, _ = reduced(A, B)
    As, Bs = Ar[np.ix_(STATE_IDX, STATE_IDX)], Br[STATE_IDX, :]
    n = len(STATE_IDX)
    # integrators of [z, phi, theta] errors: states 0, 1, 2 of the reduced set
    Aa = np.zeros((n + 3, n + 3)); Ba = np.zeros((n + 3, 3))
    Aa[:n, :n], Ba[:n] = As, Bs
    for i in range(3):
        Aa[n + i, i] = 1.0
    Q, R = _q_r(w)
    P = solve_continuous_are(Aa, Ba, Q, R)
    K = np.linalg.solve(R, Ba.T @ P)
    return K, u0[:3].copy()


def hinge_stiffness(vp, name: str, speed: float) -> float:
    """Restoring hydrodynamic hinge stiffness -dM/d(delta) [N m/rad] of one drive's surface at level flight
    (`speed`, nominal flying depth). It grows with q c_f^2: about 4 N m/rad on the 25 kg model at 4 m/s, about
    100 N m/rad on the manned flap at 8 m/s, against the servo kp of 250, so it cannot be left out."""
    from hydrofoil.forces.foil import free_surface_lift_factor
    p = vp.params
    foil = {"left": vp.left_foil, "right": vp.right_foil, "elev": vp.elev_foil}[name]
    h = vp.hinges[name]
    depth = (p["hull_offset"] - p["flying_height"]) + foil.r_attach[2]
    lf = foil.lift_slope_factor * free_surface_lift_factor(depth / foil.geometry.chord)
    q = 0.5 * h.rho * speed * speed
    if h.g.layout == "flap":
        return -q * h._ref * lf * h.ch_delta
    return -q * h._ref * h._arm * foil._a2d * lf * foil.flap_effectiveness


def servo_model(vp, speed: float):
    """Linear model of the surface drives on the three virtual channels [main, elev, roll]:
    delta_dd = g cmd - wn2 delta - c delta_d, from the moteus position gains, the total inertia at the
    output (structure + water + reflected rotor), the hydrodynamic hinge stiffness and damping at `speed`
    (wn2 = (kp + k_hinge) / J, g = kp / J: a restoring hinge moment leaves the surface short of its command).
    The current/voltage limits are left out. Returns (wn2, c, g)."""
    wn2, c, g = [], [], []
    for name in ("left", "elev", "left"):                   # main and roll both act through the flap drives
        d = vp.drives[name]
        wn2.append((d.kp + hinge_stiffness(vp, name, speed)) / d.j_total)
        c.append((d.kd + d.h.damping(speed)) / d.j_total)
        g.append(d.kp / d.j_total)
    return np.array(wn2), np.array(c), np.array(g)


def allocate_split_flaps(delta_main: float, delta_roll: float, limit_deg: float) -> tuple[float, float, bool, bool]:
    """
    Roll-prioritized control allocation for the split front-wing flaps:
        delta_left  = delta_main - delta_roll
        delta_right = delta_main + delta_roll
    Enforces |delta_left| <= limit_deg and |delta_right| <= limit_deg (equivalently
    |delta_main| + |delta_roll| <= limit_deg) by preserving roll authority first so
    a low-speed turn or gust cannot steal roll margin and capsize the craft.
    Returns (main_alloc, roll_alloc, main_clipped, roll_clipped).
    """
    lim = float(limit_deg)
    r_alloc = float(np.clip(delta_roll, -lim, lim))
    r_clip = abs(delta_roll) > lim + 1e-12
    m_lim = max(0.0, lim - abs(r_alloc))
    m_alloc = float(np.clip(delta_main, -m_lim, m_lim))
    m_clip = abs(delta_main) > m_lim + 1e-12
    return m_alloc, r_alloc, m_clip, r_clip


def steady_turn_sensitivity(As: np.ndarray, Bs: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    First-order steady-turn equilibrium slope per rad of bank angle phi_ref from the
    linearised plant (As 8x8 on [z phi theta v w p q r], Bs 8x3 on [main elev roll]):
        [As[:, 3:8], Bs] @ [nu_turn'; u_turn'] = -As[:, phi]
    Also returns the longitudinal 2x2 sub-matrices (A_lon, B_lon_inv) on rows/cols
    (w, q) and inputs (delta_main, delta_elev) for second-order level-turn lift/pitch
    compensation (w_ref = u tan(theta) - v_ref tan(phi), q_ref = r_ref tan(phi), and
    body-z gravity cos(phi) - 1).
    """
    M_eq = np.hstack([As[:, 3:8], Bs])
    try:
        sol = np.linalg.solve(M_eq, -As[:, 1])
    except np.linalg.LinAlgError:
        try:
            sol = np.linalg.lstsq(M_eq, -As[:, 1], rcond=None)[0]
        except np.linalg.LinAlgError:
            sol = np.zeros(8)
    turn_nu = sol[:5].copy()
    turn_u = sol[5:8].copy()
    A_lon = As[np.ix_([4, 6], [4, 6])].copy()
    B_lon = Bs[np.ix_([4, 6], [0, 1])].copy()
    try:
        B_lon_inv = np.linalg.inv(B_lon)
    except np.linalg.LinAlgError:
        try:
            B_lon_inv = np.linalg.pinv(B_lon, rcond=1e-6)
        except np.linalg.LinAlgError:
            B_lon_inv = np.zeros((2, 2))
    return turn_nu, turn_u, A_lon, B_lon_inv


def _augmented_from_linear(vp, As, Bs, u0, speed: float, dt: float, delay_steps: int):
    n = len(STATE_IDX)
    wn2, c, gain = servo_model(vp, speed)
    nc = n + 9                                              # continuous part: plant, integrators, delta, delta_dot
    Ac = np.zeros((nc, nc)); Bc = np.zeros((nc, 3))
    Ac[:n, :n] = As
    Ac[:n, n + 3:n + 6] = Bs
    for i in range(3):
        Ac[n + i, i] = 1.0                                  # integrators of z, phi, theta
        Ac[n + 3 + i, n + 6 + i] = 1.0                      # d(delta)/dt = delta_dot
        Ac[n + 6 + i, n + 3 + i] = -wn2[i]
        Ac[n + 6 + i, n + 6 + i] = -c[i]
        Bc[n + 6 + i, i] = gain[i]
    M = np.zeros((nc + 3, nc + 3)); M[:nc, :nc] = Ac; M[:nc, nc:] = Bc
    E = expm(M * dt)
    Ad0, Bd0 = E[:nc, :nc], E[:nc, nc:]
    d = int(delay_steps)
    N = nc + 3 * d
    Ad = np.zeros((N, N)); Bd = np.zeros((N, 3))
    Ad[:nc, :nc] = Ad0
    if d == 0:
        Bd[:nc] = Bd0
    else:
        Ad[:nc, N - 3:] = Bd0                               # the oldest stored command reaches the drives
        Bd[nc:nc + 3] = np.eye(3)                           # the new command enters the chain
        for j in range(1, d):
            Ad[nc + 3 * j:nc + 3 * j + 3, nc + 3 * (j - 1):nc + 3 * j] = np.eye(3)
    return Ad, Bd, u0[:3].copy()


def _solve_delay_lqr(Ad, Bd, dt: float, weights: dict | None = None):
    w = dict(DEFAULT_WEIGHTS, **(weights or {}))
    N, n = Ad.shape[0], len(STATE_IDX)
    Q11, R = _q_r(w)
    Q = np.zeros((N, N)); Q[:n + 3, :n + 3] = Q11
    Rd = np.diag([1 / (w[k] * dt) ** 2 for k in ("du_main", "du_elev", "du_roll")])
    E = np.zeros((3, N)); E[:, n + 9:n + 12] = np.eye(3)    # picks c[k-1]
    Q = Q + E.T @ Rd @ E
    S = -E.T @ Rd
    Rt = R + Rd
    P = solve_discrete_are(Ad, Bd, Q, Rt, s=S)
    return np.linalg.solve(Rt + Bd.T @ P @ Bd, Bd.T @ P @ Ad + S.T)


def augmented_model(vp, speed: float, dt: float, delay_steps: int):
    """Discrete design model at one speed, sampled at the control rate, with what the plain LQR left out:

        x = [ z phi theta v w p q r | int z, int phi, int theta | delta (3) | delta_dot (3) | c[k-1] ... c[k-d] (3 d) ]

    delta are the ACHIEVED surface angles (deviation from trim, deg), c the commands. The plant is driven by
    delta, the drives by the command issued `delay_steps` samples ago. Returns (Ad, Bd, trim angles)."""
    x0, u0 = trim(vp, speed)
    A, B = linearize(vp, x0, u0)
    Ar, Br, _ = reduced(A, B)
    As, Bs = Ar[np.ix_(STATE_IDX, STATE_IDX)], Br[STATE_IDX, :]
    return _augmented_from_linear(vp, As, Bs, u0, speed, dt, delay_steps)


def design_delay_aware(vp, speed: float, dt: float, delay_steps: int, weights: dict | None = None):
    """Discrete LQR on `augmented_model`: K (3 x (17 + 3 d)) and the trim angles. On top of the Bryson
    weights it penalises the command RATE (c[k] - c[k-1]) / dt, which is what keeps the loop bandwidth
    below the drive and latency instead of fighting them."""
    d = max(int(delay_steps), 1)                            # c[k-1] is needed for the rate penalty
    Ad, Bd, T = augmented_model(vp, speed, dt, d)
    return _solve_delay_lqr(Ad, Bd, dt, weights), T


_NON_PLANT_KEYS = frozenset({
    "controller", "lqr_weights", "sensors", "supervisor", "supervisor_tuning",
    "waves_enabled", "wave_amplitude", "wave_wavelength", "wave_direction",
    "phi_ref", "theta_ref", "target_speed", "kp_heave", "ki_heave", "kd_heave",
    "kp_pitch", "ki_pitch", "kd_pitch", "kp_roll", "ki_roll", "kd_roll",
    "kp_speed", "ki_speed", "kd_speed", "t_end", "tuned_for", "phase_controllers",
    "default_payloads", "lqr_speeds", "lqr_engage_speed", "lqr_delay_s",
    "allocator", "safety_margin", "reverse_drive_enabled", "flap_cmd_max_deg",
})
_PLANT_LIN_CACHE: dict[tuple, list] = {}


def _freeze_val(v):
    if isinstance(v, dict):
        return tuple((k, _freeze_val(v[k])) for k in sorted(v))
    if isinstance(v, (list, tuple)):
        return tuple(_freeze_val(x) for x in v)
    if isinstance(v, np.ndarray):
        return tuple(v.ravel().tolist())
    if isinstance(v, (np.floating, float)):
        return float(v)
    if isinstance(v, (np.integer, int)):
        return int(v)
    return v


def _plant_cache_key(vp, speeds, dt: float, delay_steps: int) -> tuple:
    p_key = tuple((k, _freeze_val(vp.params[k])) for k in sorted(vp.params) if k not in _NON_PLANT_KEYS)
    drv_key = tuple((n, float(vp.drives[n].kp), float(vp.drives[n].kd), float(vp.drives[n].j_total))
                    for n in ("left", "right", "elev")) if getattr(vp, "drives", None) else ()
    att_key = tuple(round(float(x), 9) for x in vp.left_foil.r_attach)
    return (p_key, drv_key, att_key, tuple(float(s) for s in speeds), float(dt), int(delay_steps))


class LQRController:
    """Scheduled, delay-aware LQR on the flap channels with integral action and a latched, bumpless PID
    handover. The gains come from `design_delay_aware`: the feedback uses the plant state, the achieved
    surface angles and rates (drive encoders) and its own last `delay_steps` commands."""

    def __init__(self, vp, dt: float, speeds=(3.0, 4.0, 5.0, 6.0, 7.0), weights: dict | None = None,
                 engage_speed: float = 3.2, engage_height_err: float = 0.15, limit_deg: float = 20.0,
                 int_limit=(0.05, 0.03, 0.03), delay_s: float = 0.01, blend_s: float = 0.5,
                 auto_handback: bool = False):
        self.dt, self.limit, self.blend_s = dt, limit_deg, blend_s
        self.auto_handback = bool(auto_handback)
        self.disengage_speed = 0.85 * float(engage_speed)
        self._last_draft = 0.0
        # Physical per-surface split-flap diamond ceiling: keeps left = main - roll and
        # right = main + roll inside the mechanical stop without starving heave during wave
        # crests when small roll corrections (~1-4 deg) coincide with full +-20 deg heave.
        self.phys_limit = min(float(vp.params.get("flap_stop_deg", 30.0)), limit_deg + 4.0)
        # per-phase selection (control/supervisor.py, params["phase_controllers"]): None = the automatic handover
        # (engage on speed and height error, latch); True = engage now whatever the state; False = PID, and if the
        # LQR was flying, fade back to the PID's command over blend_s
        self.force: bool | None = None
        self.hold_heave: bool = False                      # True when an outer mode (e.g. DISPLACEMENT) overrides delta_main
        self._release_t, self._release_from = 0.0, None
        self.speeds = np.asarray(speeds, float)
        self.delay_steps = max(int(round(delay_s / dt)), 1)
        self.mass_ratio_z = float(vp.rigid_body.mass / vp.rigid_body.M[2, 2])
        key = _plant_cache_key(vp, self.speeds, dt, self.delay_steps)
        cached = _PLANT_LIN_CACHE.get(key)
        if cached is None:
            cached, ok_speeds = [], []
            for s in self.speeds:
                try:
                    x0, u0 = trim(vp, float(s))
                except RuntimeError:
                    continue                    # no level trim at this speed (e.g. too slow for the wing): skip it
                if abs(float(u0[0])) > limit_deg or abs(float(u0[1])) > limit_deg:
                    continue                    # "trim" past the surfaces' command limit is not a flight condition
                A, B = linearize(vp, x0, u0)
                Ar, Br, _ = reduced(A, B)
                As, Bs = Ar[np.ix_(STATE_IDX, STATE_IDX)], Br[STATE_IDX, :]
                Ad, Bd, T = _augmented_from_linear(vp, As, Bs, u0, float(s), dt, self.delay_steps)
                tnu, tu, alon, blinv = steady_turn_sensitivity(As, Bs)
                cached.append((float(s), (Ad, Bd, T, tnu, tu, alon, blinv)))
                ok_speeds.append(float(s))
            if not ok_speeds:
                raise RuntimeError(f"no level trim inside the surface limits at any scheduled speed "
                                   f"{tuple(float(s) for s in self.speeds)}")
            if len(_PLANT_LIN_CACHE) >= 32:
                _PLANT_LIN_CACHE.clear()
            _PLANT_LIN_CACHE[key] = cached
        self.speeds = np.asarray([s for s, _ in cached], float)
        cached = [c for _, c in cached]
        if self.speeds[0] > float(engage_speed):            # never engage below the slowest trimmed design speed
            engage_speed = float(self.speeds[0])
            self.disengage_speed = 0.85 * engage_speed
        ks, ts, t_nus, t_us, t_alons, t_blins = [], [], [], [], [], []
        for Ad, Bd, T, tnu, tu, alon, blinv in cached:
            ks.append(_solve_delay_lqr(Ad, Bd, dt, weights))
            ts.append(T.copy())
            t_nus.append(tnu.copy()); t_us.append(tu.copy())
            t_alons.append(alon.copy()); t_blins.append(blinv.copy())
        self.K = np.asarray(ks)                                  # (ns, 3, 17 + 3 d)
        self.trim = np.asarray(ts)                               # (ns, 3)
        self.turn_nu = np.asarray(t_nus)                         # (ns, 5) [v w p q r] / rad bank
        self.turn_u = np.asarray(t_us)                           # (ns, 3) [main elev roll] deg / rad bank
        self.turn_A_lon = np.asarray(t_alons)                    # (ns, 2, 2)
        self.turn_B_lon_inv = np.asarray(t_blins)                # (ns, 2, 2)
        self.engage_speed, self.engage_err = engage_speed, engage_height_err
        self.int_limit = np.asarray(int_limit)                   # |integral| clamps [m s, rad s, rad s]
        self.reset()

    def reset(self):
        self.engaged = False
        self.since_engage = 0.0
        self._release_t, self._release_from = 0.0, None
        self._last_draft = 0.0
        self.integ = np.zeros(3)
        self.last = np.zeros(3)
        self.hist = np.zeros((self.delay_steps, 3))              # commands already sent, newest first [deg]
        self._last_err = np.zeros(3)
        self._last_Ki = None
        self._last_cmd_nom = np.zeros(3)
        self._last_integ_step = np.zeros(3)

    def status_dict(self, u: float) -> dict:
        """Human-readable controller handover state and reason for telemetry and UI presentation."""
        if self.force is False:
            if self._release_from is not None:
                a = min(1.0, self._release_t / self.blend_s) if self.blend_s > 0 else 1.0
                return {"engaged": False, "blending_in": False, "blending_out": True,
                        "active": "LQR → PID", "blend": float(1.0 - a),
                        "reason": "Settling back to displacement — fading LQR smoothly into PID"}
            return {"engaged": False, "blending_in": False, "blending_out": False,
                    "active": "PID", "blend": 0.0,
                    "reason": "PID active (three independent height, pitch, and roll SISO loops)"}
        if self.engaged:
            if self.since_engage < self.blend_s and self.blend_s > 0:
                a = min(1.0, self.since_engage / self.blend_s)
                return {"engaged": True, "blending_in": True, "blending_out": False,
                        "active": "PID → LQR", "blend": float(a),
                        "reason": "Hull clear & flying — smoothly handing over from PID to multivariable LQR"}
            return {"engaged": True, "blending_in": False, "blending_out": False,
                    "active": "LQR", "blend": 1.0,
                    "reason": "Foilborne cruise & turns — speed-scheduled MIMO LQR stabilizes heave, pitch, roll & yaw"}
        if self._release_from is not None:
            a = min(1.0, self._release_t / self.blend_s) if self.blend_s > 0 else 1.0
            return {"engaged": False, "blending_in": False, "blending_out": True,
                    "active": "LQR → PID", "blend": float(1.0 - a),
                    "reason": "Slowing down / hull touching water — fading LQR smoothly back to PID"}
        return {"engaged": False, "blending_in": False, "blending_out": False,
                "active": "PID (arming LQR)" if u < self.engage_speed else "PID (waiting for height settle)",
                "blend": 0.0,
                "reason": "Hull in water or below LQR handover speed — PID handles displacement and takeoff"}

    def sync_sent(self, sent_vec, sat_flags=None):
        """Overwrite the most recently recorded command with what was actually sent after shaping/slew limits,
        and roll back the outer integral step on channels saturated by the downstream allocator."""
        v = np.asarray(sent_vec, float)
        self.last = v
        self.hist[0] = v
        if sat_flags is not None and self.engaged and self._last_Ki is not None:
            Ki = self._last_Ki
            err = self._last_err
            cmd_nom = self._last_cmd_nom
            for ch, j in ((0, 0), (1, 2), (2, 1)):  # cmd/sat [main, elev, roll] <-> err/integ [z, phi, theta]
                if not sat_flags[ch] or abs(self._last_integ_step[j]) <= 0.0:
                    continue
                if abs(cmd_nom[ch] - v[ch]) <= 0.15:
                    continue
                sat_hi = (cmd_nom[ch] > v[ch]) and (Ki[ch, j] * err[j] < 0.0)
                sat_lo = (cmd_nom[ch] < v[ch]) and (Ki[ch, j] * err[j] > 0.0)
                if sat_hi or sat_lo:
                    self.integ[j] = float(np.clip(self.integ[j] - self._last_integ_step[j], -self.int_limit[j], self.int_limit[j]))
            self._last_integ_step[:] = 0.0

    def _sched_weights(self, u: float):
        if len(self.speeds) == 1:
            return 0, 0, 0.0
        u = float(np.clip(u, self.speeds[0], self.speeds[-1]))
        j = int(np.clip(np.searchsorted(self.speeds, u) - 1, 0, len(self.speeds) - 2))
        f = (u - self.speeds[j]) / (self.speeds[j + 1] - self.speeds[j])
        return j, j + 1, f

    def _sched(self, u: float):
        j, k, f = self._sched_weights(u)
        return (1 - f) * self.K[j] + f * self.K[k], (1 - f) * self.trim[j] + f * self.trim[k]

    def _turn_reference(self, u: float, phi_ref: float, theta_ref: float, T: np.ndarray):
        """
        Steady-turn equilibrium velocities nu_ref = [v, w, p, q, r] and flap trim T_turn
        for banked flight at (u, phi_ref, theta_ref). Identically (zeros(5), T) when
        phi_ref == 0 and theta_ref == 0, so straight-line flight is unchanged.
        """
        if abs(phi_ref) < 1e-6 and abs(theta_ref) < 1e-6:
            return np.zeros(5), T
        j, k, f = self._sched_weights(u)
        tnu = (1 - f) * self.turn_nu[j] + f * self.turn_nu[k]
        tu = (1 - f) * self.turn_u[j] + f * self.turn_u[k]
        alon = (1 - f) * self.turn_A_lon[j] + f * self.turn_A_lon[k]
        blinv = (1 - f) * self.turn_B_lon_inv[j] + f * self.turn_B_lon_inv[k]
        tphi = float(np.tan(np.clip(phi_ref, -np.radians(35.0), np.radians(35.0))))
        tth = float(np.tan(np.clip(theta_ref, -np.radians(20.0), np.radians(20.0))))
        v_ref = float(tnu[0]) * tphi
        r_ref = float(tnu[4]) * tphi
        w_ref = float(u) * tth - v_ref * tphi                    # level flight z_dot = 0
        q_ref = r_ref * tphi                                     # horizontal turn kinematics
        nu_ref = np.array([v_ref, w_ref, 0.0, q_ref, r_ref])
        # Second-order longitudinal trim shift: A_lon already includes +(m/M_zz)*u*q from C_RB,
        # so only the nonlinear body-z gravity drop (cos(phi) - 1) is added externally.
        dw_grav = self.mass_ratio_z * G * (float(np.cos(phi_ref)) - 1.0)
        du_lon = -blinv @ (alon @ np.array([w_ref, q_ref]) + np.array([dw_grav, 0.0]))
        du_turn = np.array([float(du_lon[0]), float(du_lon[1]), float(tu[2]) * tphi])
        return nu_ref, T + du_turn

    def _push(self, cmd):
        self.hist = np.vstack([cmd, self.hist[:-1]])

    def update(self, state, z_ref: float, phi_ref: float, theta_ref: float, pid_raw: dict,
               act=None, draft: float = 0.0) -> dict:
        """Raw virtual-channel commands [deg]. `pid_raw` is what the PIDs (incl. feed-forward) would command;
        `act` = (achieved [main, elev, roll] angles [deg], their rates [deg/s]) from the drives (None: assume
        the drives sit on the last command)."""
        eta, nu = state[:6], state[6:]
        z, phi, th = eta[2], eta[3], eta[4]
        u = nu[0]
        self._last_draft = float(draft)
        pid_vec = np.array([pid_raw["delta_main"], pid_raw["delta_elev"], pid_raw["delta_roll"]])
        K, T = self._sched(u)
        nu_ref, T_eq = self._turn_reference(u, phi_ref, theta_ref, T)
        err = np.array([z - z_ref, phi - phi_ref, th - theta_ref])
        xs = np.concatenate([err, nu[1:6] - nu_ref])             # z phi theta | v w p q r (about turn equilibrium)
        ang, rate = (self.hist[0], np.zeros(3)) if act is None else (np.asarray(act[0], float), np.asarray(act[1], float))
        tail = np.concatenate([ang - T_eq, rate, (self.hist - T_eq).ravel()])   # drives and the command chain
        Ki, Kt = K[:, 8:11], K[:, 11:]

        if self.force is False and self.engaged:          # hand back to the PID: fade from the last LQR command
            self.engaged = False
            self._release_t, self._release_from = 0.0, self.last.copy()
        elif self.auto_handback and self.force is None and self.engaged:
            landing_touch = draft > 0.02 and (u < self.engage_speed or z_ref >= z - 0.01)
            if u < self.disengage_speed or landing_touch:
                self.engaged = False
                self._release_t, self._release_from = 0.0, self.last.copy()
                self.integ[:] = 0.0
        if not self.engaged:
            want_auto = u >= self.engage_speed and abs(err[0]) < self.engage_err and (not self.auto_handback or draft <= 1e-3)
            want = (self.force is True) or (self.force is None and want_auto)
            if want:
                self._release_from = None
                self.since_engage = 0.0
                self.engaged = True
                # bumpless: choose the three integrators so the first LQR command equals the PIDs' on every
                # channel (K_i integ = base - pid); clipped, so a large mismatch still leaves a small step
                base = T_eq - K[:, :8] @ xs - Kt @ tail
                try:
                    self.integ = np.linalg.solve(Ki, base - pid_vec)
                except np.linalg.LinAlgError:
                    self.integ = np.zeros(3)
                self.integ = np.clip(self.integ, -self.int_limit, self.int_limit)
                if self.hold_heave:
                    self.integ[0] = 0.0
            else:
                out = pid_vec
                if self._release_from is not None:
                    self._release_t += self.dt
                    a = min(1.0, self._release_t / self.blend_s) if self.blend_s > 0 else 1.0
                    out = a * pid_vec + (1.0 - a) * self._release_from
                    if a >= 1.0:
                        self._release_from = None
                self.last = out
                self._push(out)
                return {"delta_main": float(out[0]), "delta_elev": float(out[1]), "delta_roll": float(out[2])}

        cmd_nom = T_eq - K[:, :8] @ xs - Ki @ self.integ - Kt @ tail
        # handover blend: the integrators cannot always absorb the PID/LQR mismatch (they are clamped), so the
        # command fades from the PID's to the LQR's over blend_s instead of stepping (a 14 deg elevator step with
        # the autotuned weights, 2026-11-15)
        self.since_engage += self.dt
        a = min(1.0, self.since_engage / self.blend_s) if self.blend_s > 0 else 1.0
        if a < 1.0:
            cmd_nom = a * cmd_nom + (1.0 - a) * pid_vec
        # Roll-prioritized physical split-flap allocation: enforces +-limit_deg on each virtual
        # channel AND guarantees both left = main - roll and right = main + roll stay inside
        # +-phys_limit (inside the mechanical stop) while preserving roll authority first.
        m_pre = float(np.clip(cmd_nom[0], -self.limit, self.limit))
        r_pre = float(np.clip(cmd_nom[2], -self.limit, self.limit))
        m_alloc, r_alloc, m_dclip, r_dclip = allocate_split_flaps(m_pre, r_pre, self.phys_limit)
        m_clip = m_dclip or abs(cmd_nom[0]) > self.limit + 1e-12
        r_clip = r_dclip or abs(cmd_nom[2]) > self.limit + 1e-12
        e_alloc = float(np.clip(cmd_nom[1], -self.limit, self.limit))
        cmd = np.array([m_alloc, e_alloc, r_alloc])
        # conditional integration: hold an integrator while its channel is saturated (including physical
        # split-flap diamond saturation on main) and the error pushes deeper
        self._last_integ_step[:] = 0.0
        for ch, j in ((0, 0), (1, 2), (2, 1)):
            if j == 0 and self.hold_heave:
                continue
            clipped = m_clip if ch == 0 else (abs(cmd_nom[1]) > self.limit + 1e-12 if ch == 1 else r_clip)
            sat_hi = clipped and (cmd_nom[ch] > cmd[ch]) and K[ch, 8 + j] * err[j] < 0
            sat_lo = clipped and (cmd_nom[ch] < cmd[ch]) and K[ch, 8 + j] * err[j] > 0
            if not (sat_hi or sat_lo):
                prev_i = float(self.integ[j])
                new_i = float(np.clip(prev_i + err[j] * self.dt, -self.int_limit[j], self.int_limit[j]))
                self.integ[j] = new_i
                self._last_integ_step[j] = new_i - prev_i
        self._last_err = err.copy()
        self._last_Ki = Ki
        self._last_cmd_nom = cmd_nom.copy()
        self.last = cmd
        self._push(cmd)
        return {"delta_main": float(cmd[0]), "delta_elev": float(cmd[1]), "delta_roll": float(cmd[2])}
