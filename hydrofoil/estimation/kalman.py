"""
Speed-scheduled steady-state Kalman filter on the linearised plant (hydrofoil/linearize.py).

State estimated: [z phi theta v w p q r] (surge u comes straight from the speed sensor). The model is the
foilborne plant linearised at trim for a few speeds, discretised at the control rate; the filter gain is the
steady-state (DARE) gain per speed, interpolated in the measured speed. Inputs are the ACHIEVED flap angles
relative to trim. Measurements are whatever arrived this step (z, phi, theta, p, q, r); missing sensors are
simply skipped. Height and heave velocity come from `HeaveFilter` (below), which handles the slow, late and
wave-contaminated ranging sensor; this filter receives that height as its z measurement every step.
"""

from __future__ import annotations

import numpy as np
from scipy.linalg import expm, solve_discrete_are

from hydrofoil.linearize import trim, linearize, reduced

STATE_IDX = [0, 1, 2, 4, 5, 6, 7, 8]                       # reduced [z phi theta u v w p q r] without u
MEAS_NAMES = ("z", "phi", "theta", "p", "q", "r")
MEAS_STATE = {"z": 0, "phi": 1, "theta": 2, "p": 5, "q": 6, "r": 7}


def _discretise(A, B, dt):
    n, m = B.shape
    M = np.zeros((n + m, n + m)); M[:n, :n] = A; M[:n, n:] = B
    E = expm(M * dt)
    return E[:n, :n], E[:n, n:]


class KalmanEstimator:
    def __init__(self, vp, dt: float, speeds=(3.0, 4.0, 5.0, 6.0, 7.0), sigmas: dict | None = None,
                 process: dict | None = None):
        d = np.radians
        s = {"z": 0.01, "phi": d(0.3), "theta": d(0.3), "p": d(0.2), "q": d(0.2), "r": d(0.2)}
        s.update(sigmas or {})
        # process noise std per sqrt(s) on the states (unmodelled forces / waves); PLACEHOLDER tuning. Too small on the
        # angles/height and the filter rejects real attitude changes (3 deg pitch kick took 4 s); too large and noise
        # leaks into the flaps (see docs/19).
        q = {"z": 3e-3, "phi": 5e-3, "theta": 5e-3, "v": 0.3, "w": 0.15, "p": 0.5, "q": 0.5, "r": 0.3}
        q.update(process or {})
        self.dt, self.speeds = dt, np.asarray(speeds, float)
        Rv = np.diag([s[k] ** 2 for k in MEAS_NAMES])
        H = np.zeros((6, 8))
        for i, k in enumerate(MEAS_NAMES):
            H[i, MEAS_STATE[k]] = 1.0
        Qw = np.diag([q[k] ** 2 * dt for k in ("z", "phi", "theta", "v", "w", "p", "q", "r")])
        Ads, Bds, Ls, trims, ok_speeds = [], [], [], [], []
        limit_deg = max(float(vp.params.get("flap_cmd_max_deg", 20.0)), float(vp.params.get("elev_cmd_max_deg", 20.0)))
        for sp in self.speeds:
            try:
                x0, u0 = trim(vp, float(sp))
            except RuntimeError:
                continue
            if abs(float(u0[0])) > limit_deg or abs(float(u0[1])) > limit_deg:
                continue
            A, B = linearize(vp, x0, u0)
            Ar, Br, _ = reduced(A, B)
            Ad, Bd = _discretise(Ar[np.ix_(STATE_IDX, STATE_IDX)], Br[STATE_IDX, :], dt)
            P = solve_discrete_are(Ad.T, H.T, Qw, Rv)
            L = P @ H.T @ np.linalg.inv(H @ P @ H.T + Rv)
            Ads.append(Ad); Bds.append(Bd); Ls.append(L); trims.append(u0[:3]); ok_speeds.append(float(sp))
        if not ok_speeds:
            raise RuntimeError(f"no level trim inside surface limits for KalmanEstimator at {tuple(float(s) for s in self.speeds)}")
        self.speeds = np.asarray(ok_speeds, float)
        self.Ad, self.Bd, self.L, self.trim = map(np.asarray, (Ads, Bds, Ls, trims))
        self.H = H
        self._z_trim0 = float(vp.params["hull_offset"] - vp.params["flying_height"])
        self.reset()

    def reset(self):
        self.x = np.zeros(8)

    def _interp(self, arr, u):
        if len(self.speeds) == 1:
            return arr[0]
        u = float(np.clip(u, self.speeds[0], self.speeds[-1]))
        j = int(np.clip(np.searchsorted(self.speeds, u) - 1, 0, len(self.speeds) - 2))
        f = (u - self.speeds[j]) / (self.speeds[j + 1] - self.speeds[j])
        return (1 - f) * arr[j] + f * arr[j + 1]

    def init(self, meas: dict, z_trim: float | None = None):
        """Start from the first measurements (unmeasured v, w at 0). State is kept relative to the
        filter's linearisation depth datum `_z_trim0` so setpoint changes never perturb the model."""
        z0 = self._z_trim0
        for k in MEAS_NAMES:
            if k in meas:
                self.x[MEAS_STATE[k]] = meas[k] - (z0 if k == "z" else 0.0)

    def update(self, meas: dict, u_cmd, u_speed: float, z_trim: float | None = None) -> np.ndarray:
        """One predict + update. u_cmd = achieved [main, elev, roll] deg. Returns estimate [z phi theta v w p q r]
        in absolute coordinates (z in NED, angles/rates in rad)."""
        z0 = self._z_trim0
        Ad, Bd, L, T = (self._interp(a, u_speed) for a in (self.Ad, self.Bd, self.L, self.trim))
        self.x = Ad @ self.x + Bd @ (np.asarray(u_cmd, float) - T)
        y = np.array([meas.get(k, np.nan) - (z0 if k == "z" else 0.0) for k in MEAS_NAMES])
        ok = ~np.isnan(y)
        if ok.any():
            innov = np.where(ok, y - self.H @ self.x, 0.0)
            self.x = self.x + (L * ok[None, :]) @ innov
        out = self.x.copy()
        out[0] += z0
        return out


class HeaveFilter:
    """
    Height / vertical-speed estimator that does not use the boat model: the accelerometer drives the motion and
    the ranging sensor corrects it (the classic hydrofoil height loop).

        x = [ z, vz, b_a, xi, eta_w ]
        z' = vz,  vz' = a_meas - b_a                      (kinematics; b_a = accelerometer bias, random walk)
        xi' = eta_w,  eta_w' = -w0^2 xi - 2 zeta w0 eta_w + noise
                                                          (wave elevation under the sensor as BAND-PASS noise,
                                                           transfer s / (s^2 + 2 zeta w0 s + w0^2): no content at
                                                           zero frequency, so a steady height offset is never
                                                           mistaken for a wave)
        h_meas = z + eta_w                                (the ranging sensor sees the local surface)

    So the slow part of the range reading is taken as real height, the wave-band part as the surface moving, and
    the boat's own fast motion comes from the accelerometer. z is then height relative to the MEAN surface, which
    is what the controller holds ("platforming"). Because no mass, CG or foil data enter, it is unaffected by a
    rider or payload. Height samples are used only when new. A sample is `latency` old, so it is modelled as
    the state a moment ago, z - vz L + eta_w - eta_w' L (first order in L). Samples further than `gate`
    standard deviations from the prediction are dropped as glitches; after `max_reject` in a row the filter
    accepts that it is the one that is wrong, widens its height uncertainty and takes the sample. wave = {"w0": rad/s, "zeta": -, "sigma": m}; sigma = 0 switches the wave state off
    (the range reading is then taken as height, i.e. the boat follows every wave).
    bias_walk is deliberately ~10x the sensor's own figure: replaying simulated flights, the matched value let a
    slow drift build up (25 mm rms in calm water against 7 mm with 0.02).
    """

    def __init__(self, dt: float, sigma_h: float = 0.01, sigma_a: float = 0.05, bias_walk: float = 0.02,
                 latency: float = 0.05, wave: dict | None = None, gate: float = 5.0, max_reject: int = 4):
        w = dict({"w0": 4.0, "zeta": 1.0, "sigma": 0.10}, **(wave or {}))
        w["w0"] = max(float(w["w0"]), 1e-3)
        self.dt, self.gate, self.wave, self.max_reject = dt, gate, w, max_reject
        Aw = np.array([[0.0, 1.0], [-w["w0"] ** 2, -2.0 * w["zeta"] * w["w0"]]])
        F = np.eye(5)
        F[0, 1], F[0, 2], F[1, 2] = dt, -0.5 * dt * dt, -dt
        F[3:, 3:] = expm(Aw * dt)
        self.F, self.G = F, np.array([0.5 * dt * dt, dt, 0.0, 0.0, 0.0])
        Q = np.outer(self.G, self.G) * sigma_a ** 2
        Q[0, 0] += 1e-10
        Q[2, 2] += bias_walk ** 2 * dt
        Q[4, 4] += 4.0 * w["zeta"] * w["w0"] * w["sigma"] ** 2 * dt          # stationary std of eta_w = sigma
        self.Q, self.R = Q, sigma_h ** 2
        on = 1.0 if w["sigma"] > 0 else 0.0
        self.H = np.array([1.0, -latency, 0.0, on * latency * w["w0"] ** 2,
                           on * (1.0 + 2.0 * w["zeta"] * w["w0"] * latency)])
        self.H_vz = np.array([0.0, 1.0, 0.0, 0.0, 0.0])
        self.reset()

    def reset(self):
        self.x = np.zeros(5)
        self.P = np.eye(5)
        self.ready = False
        self._rejected = 0
        self.reject_events = 0
        self.last_rejected = False

    def init(self, h: float):
        s, w0 = max(self.wave["sigma"], 1e-3), max(self.wave["w0"], 1e-3)
        self.x = np.array([h, 0.0, 0.0, 0.0, 0.0])
        self.P = np.diag([0.05 ** 2, 0.5 ** 2, 0.1 ** 2, (s / w0) ** 2, s ** 2])
        self.ready = True

    def innov_z2(self, h: float, R_val: float | None = None) -> float:
        """Normalized squared innovation (Mahalanobis^2) of a candidate range measurement vs prediction."""
        R_use = self.R if R_val is None else float(R_val)
        innov = h - float(self.H @ self.x)
        S = float(self.H @ (self.P @ self.H) + R_use)
        return (innov * innov) / max(S, 1e-12)

    def update_vz(self, vz_meas: float, sigma_vz: float = 0.06, az: float | None = None, latency: float = 0.08):
        """GNSS vertical-velocity measurement update (Joseph form); models sensor latency
        (`vz(t - L) ~= vz + L * b_a - L * az`) and updates [vz, b_a] without step-jumping z."""
        if not self.ready:
            return self.x
        Hv = np.array([0.0, 1.0, float(latency), 0.0, 0.0])
        Rv = float(sigma_vz) ** 2
        a_cur = 0.0 if az is None else float(az)
        innov = (float(vz_meas) + float(latency) * a_cur) - float(Hv @ self.x)
        PH = self.P @ Hv
        S = float(Hv @ PH) + Rv
        if S <= 1e-12:
            return self.x
        K = PH / S
        # Zero gain on position z and wave states so delayed GNSS velocity never jumps height z discontinuously
        K[0] = 0.0
        K[3:] = 0.0
        self.x = self.x + K * innov
        I_KH = np.eye(5) - np.outer(K, Hv)
        self.P = I_KH @ self.P @ I_KH.T + np.outer(K, K) * Rv
        return self.x

    def predict(self, az: float | None):
        """Propagate kinematic + wave state one control step with the accelerometer."""
        self.x = self.F @ self.x + self.G * (0.0 if az is None else az)
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x

    def update_z(self, h: float | None, fresh_h: bool, R_override: float | None = None):
        """Measurement update for an already-predicted step using a new height sample."""
        if fresh_h and h is not None:
            R_use = self.R if R_override is None else float(R_override)
            innov = h - self.H @ self.x
            PH = self.P @ self.H
            S = self.H @ PH + R_use
            if innov * innov >= self.gate ** 2 * S and self._rejected < self.max_reject:
                self._rejected += 1
                self.last_rejected = True
            else:
                if self._rejected >= self.max_reject:
                    self.reject_events += 1
                    self.P[0, 0] += innov * innov
                    PH = self.P @ self.H
                    S = self.H @ PH + R_use
                self._rejected = 0
                self.last_rejected = False
                K = PH / S
                self.x = self.x + K * innov
                I_KH = np.eye(5) - np.outer(K, self.H)
                self.P = I_KH @ self.P @ I_KH.T + np.outer(K, K) * R_use
        return self.x

    def step(self, az: float | None, h: float | None, fresh_h: bool, R_override: float | None = None):
        """One control step: propagate with the accelerometer, correct with a NEW height sample if there is one."""
        self.predict(az)
        return self.update_z(h, fresh_h, R_override=R_override)

    @property
    def z(self) -> float:
        return float(self.x[0])

    @property
    def vz(self) -> float:
        return float(self.x[1])
