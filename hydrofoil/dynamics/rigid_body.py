"""
6-DOF rigid-body equations of motion for a marine vehicle.

Follows Fossen (2011) Chapters 3–4:
    M_RB * nu_dot + C_RB(nu) * nu = tau

where:
    M_RB = rigid-body mass/inertia matrix (6x6, constant)
    C_RB = Coriolis/centripetal matrix (6x6, function of nu)
    tau  = generalized external forces and moments [N, N, N, Nm, Nm, Nm]

Diagonal hydrodynamic added mass (heave + pitch only) is folded directly
into M here via optional RigidBody constructor arguments — see
strip_added_mass_heave_pitch() below for how a caller derives them from its
lifting surfaces' geometry. This is a deliberate exception to the usual
"added by the caller" split: Fossen's nu_dot solve is against a single
combined M = M_RB + M_A, so keeping the diagonal M_A terms in the same
matrix M_RB builds gets callers a correct combined solve for free.
Off-diagonal terms and surge/sway/yaw added mass are NOT modeled — only
M[2,2] (heave) and M[4,4] (pitch). Damping (D) is still added by the caller.
"""

import numpy as np


def skew(v: np.ndarray) -> np.ndarray:
    """3x3 skew-symmetric matrix for cross product: skew(a) @ b == a x b."""
    return np.array([[ 0,    -v[2],  v[1]],
                     [ v[2],  0,    -v[0]],
                     [-v[1],  v[0],  0   ]])


def strip_added_mass_heave_pitch(strips, rho: float = 1025.0):
    """
    2D-strip-theory hydrodynamic added mass in heave and pitch, from one or
    more lifting surfaces treated as slender strips.

    Parameters
    ----------
    strips : iterable of (chord, span, x) — one entry per lifting surface,
             chord/span in [m], x = that surface's aerodynamic-center
             position along the body x-axis relative to the CoG [m]
             (Fossen convention: positive forward).
    rho    : water density [kg/m^3]

    Returns
    -------
    (added_mass_heave, added_inertia_pitch) : added mass [kg] for M[2,2],
        added inertia [kg*m^2] for M[4,4].

    Each surface's own vertical (heave) added mass is the classical 2D
    thin-airfoil/ellipse-section result, rho*pi*c^2/4 per unit span,
    integrated over that surface's span (constant chord assumed). Pitch
    added inertia treats that same per-surface added mass as a point mass
    at its fore-aft location x and sums via the parallel-axis relation
    (sum of m_a_i * x_i^2) — a lumped approximation per lifting surface, not
    a continuous strip integral along the hull's length.
    """
    added_mass_heave = 0.0
    added_inertia_pitch = 0.0
    for chord, span, x in strips:
        m_a = rho * np.pi * chord**2 / 4.0 * span
        added_mass_heave += m_a
        added_inertia_pitch += m_a * x**2
    return float(added_mass_heave), float(added_inertia_pitch)


class RigidBody:
    """
    Rigid-body mass matrix and Coriolis matrix.

    Parameters
    ----------
    mass : float        Vehicle mass [kg]
    I_b  : (3,3) array  Inertia tensor about CoG in body frame [kg·m²]
    r_g  : (3,) array   CoG offset from body origin in body frame [m]
                        Usually (0, 0, 0) if body origin is at CoG.
    added_mass_heave    : diagonal M[2,2] hydrodynamic added mass [kg],
                        typically from strip_added_mass_heave_pitch()
    added_inertia_pitch : diagonal M[4,4] hydrodynamic added inertia
                        [kg*m^2], typically from strip_added_mass_heave_pitch()
    """

    def __init__(self, mass: float, I_b: np.ndarray, r_g: np.ndarray = None,
                 added_mass_heave: float = 0.0, added_inertia_pitch: float = 0.0):
        self.mass = mass
        self.I_b = np.asarray(I_b, dtype=float)
        self.r_g = np.zeros(3) if r_g is None else np.asarray(r_g, dtype=float)
        # Diagonal hydrodynamic added mass (heave + pitch only) — see
        # strip_added_mass_heave_pitch() above. Zero by default so callers
        # that don't pass these (e.g. sim.py) get unchanged rigid-body-only
        # behavior.
        self.added_mass_heave = added_mass_heave
        self.added_inertia_pitch = added_inertia_pitch
        self._M = self._build_M()

    def _build_M(self) -> np.ndarray:
        m = self.mass
        I = self.I_b
        S_rg = skew(self.r_g)
        M = np.zeros((6, 6))
        M[:3, :3] = m * np.eye(3)
        M[:3, 3:] = -m * S_rg
        M[3:, :3] =  m * S_rg
        M[3:, 3:] = I

        # --- Hydrodynamic added mass (M_A), diagonal only ---------------------
        # Folded into the same combined mass matrix Fossen's nu_dot solve
        # uses (M = M_RB + M_A) — see module docstring. Only heave (index 2,
        # Z) and pitch (index 4, M) are modeled; surge, sway, and yaw added
        # mass are NOT modeled (these default to 0, so this is a no-op
        # unless the caller supplies them).
        M[2, 2] += self.added_mass_heave
        M[4, 4] += self.added_inertia_pitch

        return M

    @property
    def M(self) -> np.ndarray:
        """6x6 rigid-body mass matrix (constant)."""
        return self._M

    def C(self, nu: np.ndarray) -> np.ndarray:
        """
        6x6 Coriolis/centripetal matrix as a function of body velocity nu.

        Parameterization from Fossen (2011) eq. 3.46.
        """
        m = self.mass
        v1 = nu[:3]   # [u, v, w]
        v2 = nu[3:]   # [p, q, r]
        S_rg = skew(self.r_g)
        C = np.zeros((6, 6))
        C[:3, 3:] = -skew(m * v1 + m * S_rg.T @ v2)
        C[3:, :3] = -skew(m * v1 + m * S_rg.T @ v2)
        C[3:, 3:] = -skew(self.I_b @ v2)
        return C

    def nu_dot(self, nu: np.ndarray, tau: np.ndarray) -> np.ndarray:
        """
        Compute body-frame acceleration given current velocity and total force.

        Parameters
        ----------
        nu  : (6,) body-frame velocity [u, v, w, p, q, r]
        tau : (6,) total generalized force [X, Y, Z, K, M, N]

        Returns
        -------
        nu_dot : (6,) body-frame acceleration
        """
        rhs = tau - self.C(nu) @ nu
        return np.linalg.solve(self.M, rhs)
