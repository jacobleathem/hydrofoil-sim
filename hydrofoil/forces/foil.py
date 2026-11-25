"""
Hydrofoil lift and drag forces using strip theory.

Each Foil instance represents one lifting surface (main foil, elevator, rudder).
Forces are computed in the foil's local frame and returned as a 6-DOF
generalized force vector [X, Y, Z, K, M, N] in the vehicle body frame.

CL and CD tables are indexed by angle of attack (AoA) in degrees.
Load from XFLR5/XFOIL polar files, OpenFOAM, or experimental tow-tank data.
"""

from __future__ import annotations
import bisect
import math
from pathlib import Path
import numpy as np
from dataclasses import dataclass, field

from hydrofoil.constants import G, RHO_WATER

# Default polar: NACA 0009 from XFLR5 at Re=2.70e6, Mach=0, Ncrit=9
# Stored in data/foil_tables/naca0009_xflr5_re2700000.dat
_DAT_DIR = Path(__file__).parent.parent.parent / "data" / "foil_tables"

# Flap effectiveness d(alpha_effective)/d(delta): how much of a commanded
# flap/surface deflection actually shows up as angle-of-attack change, versus
# being absorbed by hinge-line flow losses. 1.0 = full effectiveness (an
# all-moving surface, where the whole panel rotates so commanded deflection
# literally IS angle of attack — no penalty).
MAIN_FOIL_FLAP_EFFECTIVENESS = 0.21   # plain flap, 40% chord, inboard half-span
REF_FLAP_FRAC, REF_FLAP_SPAN_FRAC = 0.4, 0.5   # the geometry the 0.21 above refers to


def main_flap_effectiveness(flap_frac: float, span_frac: float) -> float:
    """Flap effectiveness for another plain-flap geometry, scaled from the 0.21 reference by thin-airfoil
    theory: tau(c_f/c) (Glauert, hinge.thin_airfoil_coeffs) times the flapped span fraction (first order:
    a part-span flap changes the lift of its own strip). The 0.21 / (tau * span) ratio, about 0.55, stands
    for hinge-gap leakage, boundary-layer and 3-D part-span losses and is kept constant across geometries.
    Returns REF exactly at the reference geometry, so default behaviour is unchanged."""
    ff = float(np.clip(flap_frac, 0.0, 0.95))
    sf = float(np.clip(span_frac, 0.0, 1.0))
    if ff <= 0.0 or sf <= 0.0:
        return 0.0
    from hydrofoil.actuation.hinge import thin_airfoil_coeffs
    tau = thin_airfoil_coeffs(ff)["tau"]
    tau0 = thin_airfoil_coeffs(REF_FLAP_FRAC)["tau"]
    return MAIN_FOIL_FLAP_EFFECTIVENESS * (tau * sf) / (tau0 * REF_FLAP_SPAN_FRAC)
ELEVATOR_FLAP_EFFECTIVENESS = 1.0     # all-moving surface (whole elevator rotates)

# Oswald span-efficiency factor e, used in the induced-drag term CL^2/(pi*AR*e).
# 0.80 reflects the main foil being split into two independent halves at the
# centerline (no continuous circulation across the root) rather than one
# uninterrupted wing, which would allow a higher e.
MAIN_FOIL_OSWALD_EFFICIENCY = 0.80


@dataclass
class FoilGeometry:
    """Physical dimensions of a single lifting surface."""
    chord: float        # mean chord length [m]
    span: float         # span (one side) [m]
    aoa_offset: float = 0.0  # fixed geometric AoA offset (washout, camber) [deg]

    @property
    def area(self) -> float:
        return self.chord * self.span


class FoilTables:
    """
    Lookup tables mapping AoA (degrees) to CL and CD.

    Interpolates linearly within the polar range. AoA values outside the
    table are clamped to the table boundary — this avoids extrapolation
    artifacts but means the model does not capture post-stall lift recovery.
    Add experimental data beyond stall when available.

    Attributes
    ----------
    aoa_deg : (N,) sorted array of AoA breakpoints [deg]
    CL      : (N,) lift coefficient at each breakpoint
    CD      : (N,) drag coefficient at each breakpoint
    source  : human-readable label (foil name + Re + source)
    """

    def __init__(
        self,
        aoa_deg: np.ndarray,
        CL: np.ndarray,
        CD: np.ndarray,
        source: str = "unknown",
    ):
        self.aoa_deg = np.asarray(aoa_deg, dtype=float)
        self.CL = np.asarray(CL, dtype=float)
        self.CD = np.asarray(CD, dtype=float)
        self.source = source
        self.aoa_min = float(self.aoa_deg[0])
        self.aoa_max = float(self.aoa_deg[-1])
        self.aoa_peak_deg = float(self.aoa_deg[int(np.argmax(self.CL))]) if len(self.CL) else 14.4
        self._aoa_l = self.aoa_deg.tolist()
        self._cl_l = self.CL.tolist()
        self._cd_l = self.CD.tolist()
        self._n = len(self._aoa_l)
        self._dcl = [
            (self._cl_l[i + 1] - self._cl_l[i]) / max(self._aoa_l[i + 1] - self._aoa_l[i], 1e-12)
            for i in range(max(0, self._n - 1))
        ]
        self._dcd = [
            (self._cd_l[i + 1] - self._cd_l[i]) / max(self._aoa_l[i + 1] - self._aoa_l[i], 1e-12)
            for i in range(max(0, self._n - 1))
        ]

    def cl(self, aoa_deg: float) -> float:
        a = float(aoa_deg)
        if a <= self.aoa_min:
            return float(self._cl_l[0])
        if a >= self.aoa_max:
            return float(self._cl_l[-1])
        i = bisect.bisect_right(self._aoa_l, a) - 1
        i = 0 if i < 0 else (self._n - 2 if i >= self._n - 1 else i)
        return float(self._cl_l[i] + self._dcl[i] * (a - self._aoa_l[i]))

    def cd(self, aoa_deg: float) -> float:
        a = float(aoa_deg)
        if a <= self.aoa_min:
            return float(self._cd_l[0])
        if a >= self.aoa_max:
            return float(self._cd_l[-1])
        i = bisect.bisect_right(self._aoa_l, a) - 1
        i = 0 if i < 0 else (self._n - 2 if i >= self._n - 1 else i)
        return float(self._cd_l[i] + self._dcd[i] * (a - self._aoa_l[i]))

    def cl_cd_slopes(self, aoa_deg: float) -> tuple[float, float, float, float]:
        """Fast scalar piecewise linear (CL, CD, dCL/daoa, dCD/daoa) within table bounds."""
        a = float(aoa_deg)
        if a <= self.aoa_min:
            return float(self._cl_l[0]), float(self._cd_l[0]), 0.0, 0.0
        if a >= self.aoa_max:
            return float(self._cl_l[-1]), float(self._cd_l[-1]), 0.0, 0.0
        i = bisect.bisect_right(self._aoa_l, a) - 1
        i = 0 if i < 0 else (self._n - 2 if i >= self._n - 1 else i)
        dx = a - self._aoa_l[i]
        dcl = self._dcl[i]
        dcd = self._dcd[i]
        return float(self._cl_l[i] + dcl * dx), float(self._cd_l[i] + dcd * dx), float(dcl), float(dcd)

    @property
    def cl_slope_per_deg(self) -> float:
        """Lift slope dCL/dα near zero AoA [per degree]."""
        mask = np.abs(self.aoa_deg) <= 3.0
        if mask.sum() < 2:
            return float(np.gradient(self.CL, self.aoa_deg).mean())
        return float(np.polyfit(self.aoa_deg[mask], self.CL[mask], 1)[0])

    @classmethod
    def from_dat(cls, path: Path | str) -> "FoilTables":
        """
        Load a polar from a plain-text file with columns: alpha  CL  CD.
        Lines starting with '#' are treated as comments.
        """
        path = Path(path)
        data = []
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split()
                if len(parts) >= 3:
                    data.append([float(parts[0]), float(parts[1]), float(parts[2])])
        arr = np.array(data)
        return cls(arr[:, 0], arr[:, 1], arr[:, 2], source=path.stem)

    @classmethod
    def naca0009(cls) -> "FoilTables":
        """NACA 0009 polar from XFLR5, Re=2.70e6, Mach=0."""
        return cls.from_dat(_DAT_DIR / "naca0009_xflr5_re2700000.dat")


def free_surface_lift_factor(submergence_over_chord: float) -> float:
    """
    Empirical free-surface lift-loss factor for a fully-submerged foil
    operating at shallow depth, as a function of submergence-to-chord ratio
    h/c (foil depth below the free surface, divided by chord).

    A foil close to the free surface loses lift (the surface behaves like an
    image system that partially cancels the foil's bound circulation); the
    loss shrinks as the foil goes deeper. This is a PLACEHOLDER monotonic fit
    — not derived from free-surface panel-method or experimental data —
    tuned only to land near ~0.85 at h/c=3 (this vehicle's nominal flying
    submergence) and approach 1.0 (no loss) by h/c=10. Replace with a
    validated correction once available.
    """
    h_c = max(submergence_over_chord, 0.0)
    DECAY_LENGTH_CHORDS = 1.58   # fit: factor(3) ~= 0.85, factor(10) ~= 0.998
    return float(1.0 - np.exp(-h_c / DECAY_LENGTH_CHORDS))


def _cross(a, b):
    """3-vector cross product (np.cross has ~10x the overhead for tiny vectors; this runs ~400x per step)."""
    return np.array([a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]])


def cavitation_number(
    speed: float,
    depth_m: float,
    p_atm: float = 101325.0,
    p_vap: float = 2340.0,
    rho: float = RHO_WATER,
) -> float:
    """Local cavitation number sigma = (p_atm + rho*g*depth - p_vap) / (0.5*rho*V^2)."""
    d = max(float(depth_m), 0.0)
    q = max(0.5 * float(rho) * float(speed) ** 2, 1e-6)
    return float((p_atm + float(rho) * G * d - p_vap) / q)


def cp_min_estimate(aoa_eff_deg: float, delta_deg: float = 0.0, is_flap: bool = True) -> float:
    """Estimated attached-flow suction peak magnitude (-Cp_min >= 0) for cavitation checks."""
    a = min(abs(float(aoa_eff_deg)), 18.0)
    d = min(abs(float(delta_deg)), 30.0) if is_flap else 0.0
    return float(0.35 + 0.12 * a + 0.012 * a * a + 0.03 * d)


def cp_min_and_slope(
    aoa_eff_deg: float, delta_deg: float = 0.0, is_flap: bool = True, daoa_ddelta: float = 0.0
) -> tuple[float, float]:
    """Suction peak magnitude (-Cp_min >= 0) and chain-rule derivative wrt delta [per deg]."""
    a_raw = abs(float(aoa_eff_deg))
    a = min(a_raw, 18.0)
    d_raw = abs(float(delta_deg)) if is_flap else 0.0
    d = min(d_raw, 30.0)
    cp = 0.35 + 0.12 * a + 0.012 * a * a + 0.03 * d
    sgn_a = (1.0 if aoa_eff_deg > 0.0 else (-1.0 if aoa_eff_deg < 0.0 else 0.0)) if a_raw < 18.0 else 0.0
    sgn_d = ((1.0 if delta_deg > 0.0 else (-1.0 if delta_deg < 0.0 else 0.0)) if d_raw < 30.0 else 0.0) if is_flap else 0.0
    dcp_dd = sgn_a * (0.12 + 0.024 * a) * daoa_ddelta + 0.03 * sgn_d
    return float(cp), float(dcp_dd)


def effective_flap_and_slope(delta_deg: float, flap_stall_deg: float | None) -> tuple[float, float, float, float]:
    """
    Effective flap angle after trailing-edge separation above `flap_stall_deg`,
    matching derivative d(eff_delta)/d(delta), separation CD increment, and d(cd_sep)/d(delta).
    """
    d = float(delta_deg)
    if flap_stall_deg is None or abs(d) <= flap_stall_deg:
        return d, 1.0, 0.0, 0.0
    d0 = float(flap_stall_deg)
    sgn = 1.0 if d >= 0.0 else -1.0
    exc = abs(d) - d0
    denom = exc + 4.0
    eff = sgn * (d0 + exc - 0.55 * (exc * exc) / denom)
    deff = 1.0 - 0.55 * (exc * (exc + 8.0)) / (denom * denom)
    cd_sep = 0.00025 * exc * exc
    dcd_sep = sgn * 0.0005 * exc
    return float(eff), float(deff), float(cd_sep), float(dcd_sep)


class Foil:
    """
    A single lifting surface attached to the vehicle.

    Parameters
    ----------
    geometry    : FoilGeometry
    tables      : FoilTables
    r_attach    : (3,) position of foil aerodynamic center in body frame [m]
                  e.g. r_attach = [1.2, 0.0, -0.3] means 1.2 m forward,
                  0.3 m below body origin
    lift_axis   : (3,) unit vector in body frame along which lift acts
                  For a horizontal foil: [0, 0, -1] (lift = upward = -z_body)
    drag_axis   : (3,) unit vector along which drag acts (opposing motion)
                  Typically derived from flow direction, not set directly.
    rho         : water density [kg/m³]
    flap_effectiveness : d(alpha_eff)/d(delta) — see module-level
                  MAIN_FOIL_FLAP_EFFECTIVENESS / ELEVATOR_FLAP_EFFECTIVENESS
    oswald_efficiency  : Oswald span-efficiency e, used in induced drag
    """

    def __init__(
        self,
        geometry: FoilGeometry,
        tables: FoilTables,
        r_attach: np.ndarray,
        lift_axis: np.ndarray = np.array([0.0, 0.0, -1.0]),
        rho: float = RHO_WATER,
        flap_effectiveness: float = 1.0,
        oswald_efficiency: float = 0.80,
        normal_axis: np.ndarray | None = None,
        flap_cm_delta: float = 0.0,
        flap_stall_deg: float | None = None,
        cavitation_model: bool = False,
        flap_dy: float = 0.0,
    ):
        self.geometry = geometry
        # Spanwise (body y) offset [m] of the flap's own centroid from this panel's r_attach. The whole panel's lift
        # acts at r_attach; with a part-span flap the EXTRA force the flap makes acts where the flap is, so its roll
        # and yaw lever arms differ. 0 = flap centred on the panel (the original lumped model).
        self.flap_dy = float(flap_dy)
        self.tables = tables
        self.r_attach = np.asarray(r_attach, dtype=float)
        # normal_axis: body-frame unit vector the foil's AoA is measured along (positive
        # AoA = flow arriving from the -normal side... i.e. velocity component ALONG normal).
        # Horizontal foil: (0,0,1) (body z, down). Vertical strut: (0,1,0) (starboard), so the
        # strut's AoA is the side-slip angle and it develops a side force. lift_axis = -normal.
        if normal_axis is not None:
            n = np.asarray(normal_axis, dtype=float)
            nn = float(np.linalg.norm(n))
            if nn <= 1e-9:
                raise ValueError("normal_axis must be a non-zero 3-vector")
            self.normal = n / nn
            self.lift_axis = -self.normal
        else:
            la = np.asarray(lift_axis, dtype=float)
            ln = float(np.linalg.norm(la))
            if ln <= 1e-9:
                raise ValueError("lift_axis must be a non-zero 3-vector")
            self.lift_axis = la / ln
            self.normal = -self.lift_axis
        self.rho = rho
        # Default (1.0) is a no-penalty, all-moving-surface assumption, so
        # callers that don't pass this explicitly (e.g. sim.py) keep their
        # prior behavior unchanged. See MAIN_FOIL_FLAP_EFFECTIVENESS /
        # ELEVATOR_FLAP_EFFECTIVENESS above for this vehicle's real values.
        self.flap_effectiveness = flap_effectiveness
        self.oswald_efficiency = max(float(oswald_efficiency), 1e-4)
        # Section pitching-moment slope of the flap about the quarter chord [per rad of REAL flap
        # deflection, nose-up positive, already scaled by flap span fraction]. A flap-down force
        # increment acts aft of c/4, so it also adds a nose-down couple. 0 = none (all-moving foil).
        self.flap_cm_delta = flap_cm_delta
        self.flap_stall_deg = flap_stall_deg
        self.cavitation_model = bool(cavitation_model)
        self.last_CL = 0.0   # corrected CL of the latest generalized_force call (read by the tail downwash)
        # Aspect ratio of this panel from its own span/chord (rectangular
        # planform assumed). Each main-foil half is modeled as its own Foil
        # at its own half-span, so this is that half-panel's local AR, not
        # the un-split wing's.
        self.aspect_ratio = (float(geometry.span) / max(float(geometry.chord), 1e-4)
                             if geometry.chord > 0 and geometry.span > 0 else 0.0)
        self._a2d = tables.cl_slope_per_deg * 180.0 / np.pi   # section slope, per rad
        # Finite-wing lift-curve correction, Prandtl/Helmbold form with the SAME
        # Oswald efficiency used for induced drag:
        #     a_3D = a_2D / (1 + a_2D / (pi * e * AR))
        # a_2D is the polar's own section slope (per rad), not a hard-coded 2*pi.
        # (Previously lift used AR/(AR+2), an e = 1 form, while induced drag used
        # e = 0.80 -- the two corrections disagreed about the same wing.)
        self.lift_slope_factor = self._lift_slope_factor(self.aspect_ratio)

    def _lift_slope_factor(self, ar: float) -> float:
        if ar <= 1e-6:
            return 0.0
        return 1.0 / (1.0 + self._a2d / (np.pi * self.oswald_efficiency * ar))

    def flow(self, nu: np.ndarray, r_attach: np.ndarray | None = None) -> tuple[float, float]:
        """In-plane flow speed [m/s] and geometric angle of attack [rad, excl. flap] at the foil
        (same kinematics as generalized_force; used by the actuator load model)."""
        r = self.r_attach if r_attach is None else r_attach
        V_foil = nu[:3] + _cross(nu[3:], r)
        V_n = float(V_foil @ self.normal)
        return float(np.hypot(V_foil[0], V_n)), float(np.arctan2(V_n, V_foil[0]) + np.radians(self.geometry.aoa_offset))

    def _polar_2d(self, aoa_deg: float) -> tuple[float, float]:
        """2D section CL and CD with smooth post-stall continuation outside polar table."""
        if self.tables.aoa_min <= aoa_deg <= self.tables.aoa_max:
            return self.tables.cl(aoa_deg), self.tables.cd(aoa_deg)
        edge = self.tables.aoa_max if aoa_deg > 0.0 else self.tables.aoa_min
        cl_edge = self.tables.cl(edge)
        cd_edge = self.tables.cd(edge)
        excess = abs(aoa_deg - edge)
        blend = min(1.0, excess / 7.0)
        blend = blend * blend * (3.0 - 2.0 * blend)
        a_rad = np.radians(aoa_deg)
        cl_post = 1.15 * abs(np.sin(2.0 * a_rad)) * (1.0 if a_rad >= 0.0 else -1.0)
        cd_post = 2.0 * np.sin(a_rad) ** 2 + 0.02
        return float((1.0 - blend) * cl_edge + blend * cl_post), float((1.0 - blend) * cd_edge + blend * cd_post)

    def _polar_2d_and_slopes(self, aoa_deg: float) -> tuple[float, float, float, float]:
        """2D section CL, CD and analytical/post-stall slopes [per deg]."""
        t = self.tables
        if t.aoa_min <= aoa_deg <= t.aoa_max:
            return t.cl_cd_slopes(aoa_deg)
        cl0, cd0 = self._polar_2d(aoa_deg)
        eps = 1e-2
        clp, cdp = self._polar_2d(aoa_deg + eps)
        clm, cdm = self._polar_2d(aoa_deg - eps)
        return cl0, cd0, (clp - clm) / (2.0 * eps), (cdp - cdm) / (2.0 * eps)

    def _eval_force_and_deriv(
        self,
        nu: np.ndarray,
        delta: float = 0.0,
        submergence_over_chord: float | None = None,
        span: float | None = None,
        r_attach: np.ndarray | None = None,
        downwash_rad: float = 0.0,
        ventilation: float = 0.0,
        with_deriv: bool = False,
    ) -> dict:
        """Shared evaluation of 6-DOF generalized force and analytical delta derivative."""
        span_use = self.geometry.span if span is None else float(span)
        r_use = self.r_attach if r_attach is None else r_attach
        chord = float(self.geometry.chord)
        if span_use <= 1e-3 or chord <= 1e-4:
            return {"tau": np.zeros(6), "dtau_ddelta": np.zeros(6), "CL": 0.0, "CD": 0.0,
                    "speed": 0.0, "aoa_flow_deg": 0.0, "aoa_eff_deg": 0.0,
                    "submergence_hc": 0.0 if submergence_over_chord is None else float(submergence_over_chord),
                    "sigma": 99.0, "cp_min": 0.35}
        ar = span_use / chord
        rx, ry, rz = float(r_use[0]), float(r_use[1]), float(r_use[2])
        p_, q_, r_ = float(nu[3]), float(nu[4]), float(nu[5])
        vx = float(nu[0]) + q_ * rz - r_ * ry
        vy = float(nu[1]) + r_ * rx - p_ * rz
        vz = float(nu[2]) + p_ * ry - q_ * rx

        nx, ny, nz = float(self.normal[0]), float(self.normal[1]), float(self.normal[2])
        lx, ly, lz = float(self.lift_axis[0]), float(self.lift_axis[1]), float(self.lift_axis[2])
        V_n = vx * nx + vy * ny + vz * nz
        vpx = vx + V_n * nx
        vpy = V_n * ny
        vpz = V_n * nz
        speed = math.sqrt(vpx * vpx + vpy * vpy + vpz * vpz)

        if speed < 0.01:
            return {"tau": np.zeros(6), "dtau_ddelta": np.zeros(6), "CL": 0.0, "CD": 0.0,
                    "speed": speed, "aoa_flow_deg": 0.0, "aoa_eff_deg": 0.0,
                    "submergence_hc": 0.0 if submergence_over_chord is None else float(submergence_over_chord),
                    "sigma": 99.0, "cp_min": 0.35}

        inv_s = 1.0 / speed
        hx, hy, hz = vpx * inv_s, vpy * inv_s, vpz * inv_s
        eff_delta, deff_ddelta, cd_flap_sep, dcd_sep_ddelta = effective_flap_and_slope(delta, self.flap_stall_deg)
        aoa_flow_deg = math.degrees(math.atan2(V_n, vx) - downwash_rad) + float(self.geometry.aoa_offset)
        aoa_deg = ((aoa_flow_deg + self.flap_effectiveness * eff_delta + 180.0) % 360.0) - 180.0
        is_flap = self.flap_effectiveness < 0.99

        q = 0.5 * self.rho * speed * speed
        A = chord * span_use

        if with_deriv:
            CL_2d, CD_2d_base, dcl2d_da, dcd2d_da = self._polar_2d_and_slopes(aoa_deg)
        else:
            if self.tables.aoa_min <= aoa_deg <= self.tables.aoa_max:
                CL_2d, CD_2d_base, _, _ = self.tables.cl_cd_slopes(aoa_deg)
            else:
                CL_2d, CD_2d_base = self._polar_2d(aoa_deg)
            dcl2d_da = dcd2d_da = 0.0
        CD_2d = CD_2d_base + cd_flap_sep
        ls = self._lift_slope_factor(ar)

        wet = 1.0
        fs = 1.0
        if submergence_over_chord is not None:
            hc = float(submergence_over_chord)
            if hc <= 0.0:
                return {"tau": np.zeros(6), "dtau_ddelta": np.zeros(6), "CL": 0.0, "CD": 0.0,
                        "speed": speed, "aoa_flow_deg": aoa_flow_deg, "aoa_eff_deg": aoa_deg,
                        "submergence_hc": hc, "sigma": 99.0,
                        "cp_min": cp_min_estimate(aoa_deg, delta, is_flap=is_flap), "cav_factor": 1.0}
            if hc < 0.25:
                t_w = hc * 4.0
                wet = t_w * t_w * (3.0 - 2.0 * t_w)
            fs = 1.0 if hc > 10.0 else (1.0 - math.exp(-hc / 1.58))

        depth_m = max(float(submergence_over_chord), 0.0) * chord if submergence_over_chord is not None else 1.0
        sigma = cavitation_number(speed, depth_m, rho=self.rho)
        daoa_dd = self.flap_effectiveness * deff_ddelta
        if with_deriv:
            cp_peak, dcp_dd = cp_min_and_slope(aoa_deg, delta, is_flap=is_flap, daoa_ddelta=daoa_dd)
        else:
            cp_peak = cp_min_estimate(aoa_deg, delta, is_flap=is_flap)
            dcp_dd = 0.0

        cav_fac = 1.0
        cd_cav = 0.0
        dcav_dd = 0.0
        dcd_cav_dd = 0.0
        if self.cavitation_model and cp_peak > sigma:
            cav_exc = cp_peak - sigma
            den = cav_exc * cav_exc + 2.5
            cav_fac = 1.0 - 0.50 * (cav_exc * cav_exc) / den
            cd_cav = 0.025 * (cav_exc * cav_exc) / (cav_exc + 0.5)
            if with_deriv:
                dcav_dd = (-2.50 * cav_exc / (den * den)) * dcp_dd
                dcd_cav_dd = (0.025 * cav_exc * (cav_exc + 1.0) / ((cav_exc + 0.5) ** 2)) * dcp_dd

        vent = min(1.0, max(0.0, float(ventilation)))
        vent_fac = 1.0 - 0.55 * vent
        lift_gain = ls * fs * vent_fac
        cl_pre_cav = CL_2d * lift_gain
        CL = cl_pre_cav * cav_fac

        k_ind = max(math.pi * ar * self.oswald_efficiency, 1e-6)
        CD_induced = CL * CL / k_ind
        CD = (CD_2d + (0.08 * vent if vent > 0.0 else 0.0) + cd_cav) * wet + CD_induced

        L_mag = q * CL * A
        D_mag = q * CD * A

        dot = lx * hx + ly * hy + lz * hz
        ldx, ldy, ldz = lx - dot * hx, ly - dot * hy, lz - dot * hz
        ln = math.sqrt(ldx * ldx + ldy * ldy + ldz * ldz)
        if ln > 1e-9:
            inv_l = 1.0 / ln
            ldx *= inv_l
            ldy *= inv_l
            ldz *= inv_l

        fx = L_mag * ldx - D_mag * hx
        fy = L_mag * ldy - D_mag * hy
        fz = L_mag * ldz - D_mag * hz
        cm_fac = wet * vent_fac * cav_fac
        tau = np.array([
            fx, fy, fz,
            ry * fz - rz * fy,
            rz * fx - rx * fz + (q * A * chord * self.flap_cm_delta * math.radians(delta) * cm_fac if self.flap_cm_delta else 0.0),
            rx * fy - ry * fx,
        ], dtype=float)

        if with_deriv:
            dCL_dd = dcl2d_da * daoa_dd * lift_gain * cav_fac + cl_pre_cav * dcav_dd
            dCD_dd = (dcd2d_da * daoa_dd + dcd_sep_ddelta + dcd_cav_dd) * wet + (2.0 * CL * dCL_dd) / k_ind
            qA = q * A
            dfx = qA * (dCL_dd * ldx - dCD_dd * hx)
            dfy = qA * (dCL_dd * ldy - dCD_dd * hy)
            dfz = qA * (dCL_dd * ldz - dCD_dd * hz)
            dtau_ddelta = np.array([
                dfx, dfy, dfz,
                ry * dfz - rz * dfy,
                rz * dfx - rx * dfz + (qA * chord * self.flap_cm_delta * ((math.pi / 180.0) * cm_fac + math.radians(delta) * wet * vent_fac * dcav_dd) if self.flap_cm_delta else 0.0),
                rx * dfy - ry * dfx,
            ], dtype=float)
        else:
            dtau_ddelta = None

        if self.flap_dy and self.flap_effectiveness > 0.0:
            # Move the flap's force increment F(delta) - F(0) from r_attach to the flap centroid (dy along body y):
            # the extra moment is (0, dy, 0) x dF = (dy dFz, 0, -dy dFx). All of dF/ddelta is flap increment.
            dy = self.flap_dy
            if delta != 0.0:
                saved, self.flap_dy = self.flap_dy, 0.0
                try:
                    tau0 = self._eval_force_and_deriv(
                        nu, delta=0.0, submergence_over_chord=submergence_over_chord, span=span, r_attach=r_attach,
                        downwash_rad=downwash_rad, ventilation=ventilation, with_deriv=False)["tau"]
                finally:
                    self.flap_dy = saved
                tau[3] += dy * (tau[2] - tau0[2])
                tau[5] -= dy * (tau[0] - tau0[0])
            if dtau_ddelta is not None:
                dtau_ddelta[3] += dy * dtau_ddelta[2]
                dtau_ddelta[5] -= dy * dtau_ddelta[0]

        return {
            "tau": tau,
            "dtau_ddelta": dtau_ddelta if dtau_ddelta is not None else np.zeros(6),
            "CL": float(CL),
            "CD": float(CD),
            "speed": speed,
            "aoa_flow_deg": aoa_flow_deg,
            "aoa_eff_deg": aoa_deg,
            "submergence_hc": 10.0 if submergence_over_chord is None else float(submergence_over_chord),
            "sigma": sigma,
            "cp_min": cp_peak,
            "cav_factor": float(cav_fac),
        }

    def local_derivatives(
        self,
        nu: np.ndarray,
        delta: float = 0.0,
        submergence_over_chord: float | None = None,
        span: float | None = None,
        r_attach: np.ndarray | None = None,
        downwash_rad: float = 0.0,
        ventilation: float = 0.0,
    ) -> dict:
        """Compute local 6-DOF force/moment, analytical delta derivative, flow, and cavitation state."""
        return self._eval_force_and_deriv(
            nu, delta=delta, submergence_over_chord=submergence_over_chord,
            span=span, r_attach=r_attach, downwash_rad=downwash_rad,
            ventilation=ventilation, with_deriv=True,
        )

    def generalized_force(
        self,
        nu: np.ndarray,
        delta: float = 0.0,
        submergence_over_chord: float | None = None,
        span: float | None = None,
        r_attach: np.ndarray | None = None,
        downwash_rad: float = 0.0,
        ventilation: float = 0.0,
    ) -> np.ndarray:
        """Compute 6-DOF generalized force [X, Y, Z, K, M, N] in the vehicle body frame."""
        res = self._eval_force_and_deriv(
            nu, delta=delta, submergence_over_chord=submergence_over_chord,
            span=span, r_attach=r_attach, downwash_rad=downwash_rad,
            ventilation=ventilation, with_deriv=False,
        )
        self.last_CL = float(res["CL"])
        return res["tau"]
