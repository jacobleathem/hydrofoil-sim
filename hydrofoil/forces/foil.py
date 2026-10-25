"""
Hydrofoil lift and drag forces using strip theory.

Each Foil instance represents one lifting surface (main foil, elevator, rudder).
Forces are computed in the foil's local frame and returned as a 6-DOF
generalized force vector [X, Y, Z, K, M, N] in the vehicle body frame.

CL and CD tables are indexed by angle of attack (AoA) in degrees.
Load from XFLR5/XFOIL polar files, OpenFOAM, or experimental tow-tank data.
"""

from __future__ import annotations
from pathlib import Path
import numpy as np
from dataclasses import dataclass, field

# Default polar: NACA 0009 from XFLR5 at Re=2.70e6, Mach=0, Ncrit=9
# Stored in data/foil_tables/naca0009_xflr5_re2700000.dat
_DAT_DIR = Path(__file__).parent.parent.parent / "data" / "foil_tables"

# Flap effectiveness d(alpha_effective)/d(delta): how much of a commanded
# flap/surface deflection actually shows up as angle-of-attack change, versus
# being absorbed by hinge-line flow losses. 1.0 = full effectiveness (an
# all-moving surface, where the whole panel rotates so commanded deflection
# literally IS angle of attack — no penalty).
MAIN_FOIL_FLAP_EFFECTIVENESS = 0.21   # plain flap, 40% chord, inboard half-span
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
        self.aoa_min = self.aoa_deg[0]
        self.aoa_max = self.aoa_deg[-1]

    def cl(self, aoa_deg: float) -> float:
        a = float(np.clip(aoa_deg, self.aoa_min, self.aoa_max))
        return float(np.interp(a, self.aoa_deg, self.CL))

    def cd(self, aoa_deg: float) -> float:
        a = float(np.clip(aoa_deg, self.aoa_min, self.aoa_max))
        return float(np.interp(a, self.aoa_deg, self.CD))

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
        rho: float = 1025.0,
        flap_effectiveness: float = 1.0,
        oswald_efficiency: float = 0.80,
    ):
        self.geometry = geometry
        self.tables = tables
        self.r_attach = np.asarray(r_attach, dtype=float)
        self.lift_axis = np.asarray(lift_axis, dtype=float) / np.linalg.norm(lift_axis)
        self.rho = rho
        # Default (1.0) is a no-penalty, all-moving-surface assumption, so
        # callers that don't pass this explicitly (e.g. sim.py) keep their
        # prior behavior unchanged. See MAIN_FOIL_FLAP_EFFECTIVENESS /
        # ELEVATOR_FLAP_EFFECTIVENESS above for this vehicle's real values.
        self.flap_effectiveness = flap_effectiveness
        self.oswald_efficiency = oswald_efficiency
        # Aspect ratio of this panel from its own span/chord (rectangular
        # planform assumed). Each main-foil half is modeled as its own Foil
        # at its own half-span, so this is that half-panel's local AR, not
        # the un-split wing's.
        self.aspect_ratio = geometry.span / geometry.chord

    def generalized_force(
        self,
        nu: np.ndarray,
        delta: float = 0.0,
        submergence_over_chord: float | None = None,
    ) -> np.ndarray:
        """
        Compute 6-DOF generalized force contribution of this foil.

        Parameters
        ----------
        nu    : (6,) body-frame velocity [u, v, w, p, q, r] [m/s, rad/s]
        delta : flap/servo deflection angle commanded, BEFORE flap
                effectiveness is applied [deg]
        submergence_over_chord : this foil's depth below the free surface
                divided by its chord (h/c). None (default) skips the
                free-surface correction entirely (factor == 1.0) — callers
                that don't track submergence (e.g. sim.py) get unchanged
                behavior.

        Returns
        -------
        tau : (6,) [X, Y, Z, K, M, N] in body frame [N, N, N, Nm, Nm, Nm]
        """
        V_body = nu[:3]   # translational velocity at body origin
        omega = nu[3:]    # angular velocity

        # velocity at foil attachment point (body frame)
        V_foil = V_body + np.cross(omega, self.r_attach)
        speed = np.linalg.norm(V_foil)

        if speed < 0.01:
            return np.zeros(6)

        # flow direction (opposing velocity = drag direction)
        V_hat = V_foil / speed

        # AoA = arctan(w / u) in NED body frame.
        # Positive w (sinking) → water hits underside → positive AoA → upward lift ✓
        # Only the flap_effectiveness-scaled portion of a commanded deflection
        # actually becomes angle-of-attack — 1 deg of flap is NOT 1 deg of AoA
        # (see class docstring / module-level *_FLAP_EFFECTIVENESS constants).
        aoa_rad = np.arctan2(V_foil[2], V_foil[0])
        aoa_deg = np.degrees(aoa_rad) + self.geometry.aoa_offset + self.flap_effectiveness * delta

        q = 0.5 * self.rho * speed**2
        A = self.geometry.area

        # 1. 2D section polar (XFLR5) — section (infinite-span) data.
        CL_2d = self.tables.cl(aoa_deg)
        CD_2d = self.tables.cd(aoa_deg)

        # 2. Finite-span correction. Strip theory here uses ONE strip per
        # foil (no spanwise circulation distribution), so the 2D section CL
        # overestimates real 3D lift; scale toward the classical finite-wing
        # limit AR/(AR+2).
        CL = CL_2d * self.aspect_ratio / (self.aspect_ratio + 2.0)

        # 3. Free-surface lift loss (no-op — factor 1.0 — if the caller
        # didn't supply a known submergence).
        if submergence_over_chord is not None:
            CL *= free_surface_lift_factor(submergence_over_chord)

        # 4. Induced drag from the corrected (3D, free-surface-adjusted) CL,
        # added on top of 2D profile drag. Must come last — it depends on
        # the fully-corrected CL, not the raw 2D section CL.
        CD_induced = CL**2 / (np.pi * self.aspect_ratio * self.oswald_efficiency)
        CD = CD_2d + CD_induced

        L_mag = q * CL * A
        D_mag = q * CD * A

        # lift perpendicular to flow in the lift plane
        lift_dir = np.cross(V_hat, np.cross(self.lift_axis, V_hat))
        ln = np.linalg.norm(lift_dir)
        if ln > 1e-9:
            lift_dir = lift_dir / ln

        force = L_mag * lift_dir - D_mag * V_hat   # [N] in body frame
        moment = np.cross(self.r_attach, force)     # [Nm] about body origin

        tau = np.zeros(6)
        tau[:3] = force
        tau[3:] = moment
        return tau
