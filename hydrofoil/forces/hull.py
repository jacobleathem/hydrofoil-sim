"""
Simplified wall-sided hull buoyancy and drag, for modeling the transition
between displacement (floating) and foilborne (flying) regimes.

The original forces/buoyancy.py::buoyancy_force() assumes constant displaced
volume — correct for a vehicle that is always fully submerged, but it can't
represent a hull rising out of the water as foil lift takes over. This module
adds that: buoyancy that depends on how deep the hull currently is (its
"draft"), tapering to zero once the hull clears the surface.

Geometry convention (NED, z positive down):
    hull_bottom_z = eta[2] (CoG depth) - hull_offset_above_cog
so a positive hull_offset_above_cog means the hull rides above the CoG
reference point (as it does on a strut/mast-mounted foil vehicle). draft is
how far the hull bottom sits below the calm-water surface (z=0), clamped to
[0, max_draft] — a linear (wall-sided) approximation valid near the design
waterline, not a full hull-form model.
"""

import numpy as np

from hydrofoil.constants import RHO_WATER, G
from hydrofoil.dynamics.kinematics import R_body_to_ned


def hull_draft(eta: np.ndarray, hull_offset_above_cog: float, max_draft: float,
                water_surface_z: float = 0.0) -> float:
    """
    Depth of the hull bottom below the (possibly wave-displaced) local water
    surface [m], clamped to [0, max_draft]. water_surface_z is the surface's
    current NED z-position (0 = calm water datum; negative = surface risen).
    """
    # Hull-bottom reference point sits hull_offset above the CoG along body -z, so its
    # NED depth follows the attitude (previously attitude was ignored here while
    # the foil submergence calculation did rotate -- inconsistent).
    R = R_body_to_ned(eta[3], eta[4], eta[5])
    hull_bottom_z = eta[2] + (R @ np.array([0.0, 0.0, -hull_offset_above_cog]))[2]
    return float(np.clip(hull_bottom_z - water_surface_z, 0.0, max_draft))


def hull_buoyancy_force(
    eta: np.ndarray,
    hull_offset_above_cog: float,
    waterplane_area: float,
    max_draft: float,
    rho_water: float = RHO_WATER,
    g: float = G,
    water_surface_z: float = 0.0,
) -> np.ndarray:
    """
    Buoyancy force that grows linearly with draft and vanishes once the hull
    is clear of the water — replaces constant-volume buoyancy near the surface.
    """
    draft = hull_draft(eta, hull_offset_above_cog, max_draft, water_surface_z)
    volume = waterplane_area * draft

    phi, theta, psi = eta[3], eta[4], eta[5]
    R = R_body_to_ned(phi, theta, psi)

    F_b = rho_water * g * volume
    f_ned = np.array([0.0, 0.0, -F_b])   # upward = -z in NED
    f_body = R.T @ f_ned

    # Buoyancy acts at the centre of buoyancy of the submerged volume, which sits
    # above the CoG (hull_offset to the hull bottom, plus half the draft). The
    # force is not through the CoG, so tilting the hull produces a restoring
    # moment r_cb x F (previously applied at the CoG -> no hydrostatic stiffness).
    r_cb = np.array([0.0, 0.0, -(hull_offset_above_cog + 0.5 * draft)])
    tau = np.zeros(6)
    tau[:3] = f_body
    tau[3:] = np.array([r_cb[1] * f_body[2] - r_cb[2] * f_body[1],
                        r_cb[2] * f_body[0] - r_cb[0] * f_body[2],
                        r_cb[0] * f_body[1] - r_cb[1] * f_body[0]])
    return tau


def hull_drag_force(
    nu: np.ndarray,
    eta: np.ndarray,
    hull_offset_above_cog: float,
    max_draft: float,
    drag_coeff: float = 15.0,
    water_surface_z: float = 0.0,
) -> np.ndarray:
    """
    Placeholder quadratic hull drag (wave-making + skin friction), scaled by
    how wetted the hull currently is (1.0 at full design draft, 0 once
    foilborne). Real hulls show a drag "hump" peaking just before takeoff;
    this linear-with-draft model captures the qualitative transition, not
    the hump's exact shape — replace with tow-tank/CFD data if precision matters.
    """
    draft = hull_draft(eta, hull_offset_above_cog, max_draft, water_surface_z)
    wetted_frac = draft / max_draft if max_draft > 0 else 0.0

    u = nu[0]
    Fx = -drag_coeff * wetted_frac * u * abs(u)

    tau = np.zeros(6)
    tau[0] = Fx
    return tau


# ---------------------------------------------------------------------------
# Distributed hull hydrostatics
# ---------------------------------------------------------------------------

class HullHydrostatics:
    """
    Hull buoyancy by integrating the local draft over the hull's footprint instead of
    lumping it at one point.

    Why: the lumped model above reproduces the heave stiffness rho*g*A_wp but misses the
    waterplane-inertia term of the metacentric height, GM = (CB above CoG) + I/V
    (Fossen 2011 ch. 4). For a 2.0 x 0.30 m box hull at its 4 cm rest draft that is
    roll stiffness 23 % low and pitch stiffness 14x too low (152 vs 2163 Nm/rad).

    Kinematics: the hull is a rigid prism whose bottom plane is hull_offset above the CoG
    along body -z. Each footprint cell i at body (x_i, y_i) has its bottom point at body
    (x_i, y_i, -hull_offset); its NED depth follows the full attitude,
        z_i = eta_z + R[2,:] . p_i
    and its local draft is clip(z_i - z_surface_i, 0, max_draft) with z_surface_i the
    (wave-displaced) surface under that cell. Each cell pushes straight up in NED with
    rho*g*dA*draft_i, i.e. f_body_i = -F_i * R[2,:], acting at the centroid of its
    submerged column, body (x_i, y_i, -(hull_offset + draft_i/2)); the moment is the sum
    of p_i x f_i. Heave, roll and pitch stiffness, partial emergence during takeoff, and
    wave slope across the hull all follow from this one integral.

    The footprint is a rectangle (length x beam, centred at x_center), a placeholder. With twin_spacing > 0 it is
    two such rectangles (catamaran pontoons, each `beam` wide) centred at y = +-twin_spacing / 2.
    """

    def __init__(self, length: float, beam: float, x_center: float, hull_offset: float,
                 max_draft: float, nx: int = 11, ny: int = 7,
                 rho: float = RHO_WATER, g: float = G, twin_spacing: float = 0.0):
        length = max(float(length), 1e-3)
        beam = max(float(beam), 1e-3)
        max_draft = max(float(max_draft), 0.0)
        nx = max(int(nx), 1)
        ny = max(int(ny), 1)
        n_hulls = 2 if float(twin_spacing) > 0.0 else 1
        xs = x_center + ((np.arange(nx) + 0.5) / nx - 0.5) * length
        ys = ((np.arange(ny) + 0.5) / ny - 0.5) * beam
        if n_hulls == 2:
            ys = np.concatenate([ys - 0.5 * float(twin_spacing), ys + 0.5 * float(twin_spacing)])
        X, Y = np.meshgrid(xs, ys, indexing="ij")
        self.xy = np.stack([X.ravel(), Y.ravel()], axis=1)                       # (N, 2)
        self.bottom = np.column_stack([self.xy, np.full(len(self.xy), -hull_offset)])   # (N, 3) body coords
        self.xy0 = self.xy.copy()
        self.bottom0 = self.bottom.copy()
        self.n_hulls = n_hulls
        self.dA = length * beam / (nx * ny)
        self.waterplane_area = n_hulls * length * beam
        self.hull_offset = float(hull_offset)
        self.hull_offset0 = float(hull_offset)
        self.max_draft = max_draft
        self.rho, self.g = rho, g
        self.n = len(self.xy)

    def set_cg_offset(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0) -> None:
        """Shift hull footprint coordinates relative to body origin when CG moves by body-NED (dx, dy, dz)."""
        off = np.array([float(dx), float(dy), float(dz)])
        self.bottom = self.bottom0 - off
        self.xy = self.bottom[:, :2]
        self.hull_offset = self.hull_offset0 + float(dz)

    def cell_world_xy(self, eta):
        """NED (x, y, z) of every cell's hull-bottom point (for sampling the wave field)."""
        R = R_body_to_ned(eta[3], eta[4], eta[5])
        return eta[:3] + self.bottom @ R.T

    def drafts(self, eta, surface_z):
        """Per-cell draft [m]; surface_z is the NED z of the surface under each cell (scalar or (N,))."""
        R = R_body_to_ned(eta[3], eta[4], eta[5])
        z_bottom = eta[2] + self.bottom @ R[2]
        return np.clip(z_bottom - surface_z, 0.0, self.max_draft)

    def mean_draft(self, eta, surface_z) -> float:
        return float(np.mean(self.drafts(eta, surface_z)))

    def buoyancy(self, eta, surface_z) -> np.ndarray:
        """6-vector [Fx Fy Fz Mx My Mz] in the body frame about the CoG."""
        R = R_body_to_ned(eta[3], eta[4], eta[5])
        r2 = R[2]                                           # NED-z row: f_body = -F * r2
        d = np.clip(eta[2] + self.bottom @ r2 - surface_z, 0.0, self.max_draft)
        F = self.rho * self.g * self.dA * d                 # (N,)
        px, py = self.xy[:, 0], self.xy[:, 1]
        pz = -(self.hull_offset + 0.5 * d)                  # centroid of each submerged column
        # moment = sum p_i x (-F_i r2)
        mx = -np.sum(F * (py * r2[2] - pz * r2[1]))
        my = -np.sum(F * (pz * r2[0] - px * r2[2]))
        mz = -np.sum(F * (px * r2[1] - py * r2[0]))
        tau = np.zeros(6)
        tau[:3] = -np.sum(F) * r2
        tau[3:] = (mx, my, mz)
        return tau

    def drag(self, nu, mean_draft: float, drag_coeff: float, rot_damping: float = 0.0,
             hull_length: float | None = None, hump_amp: float = 0.0) -> np.ndarray:
        """
        Placeholder hull damping, scaled by wetted fraction (0 once foilborne):
        quadratic surge drag with an optional wave-making Froude hump peaking near
        Fr_L = |u| / sqrt(g L) ~ 0.48 before liftoff, quadratic lateral crossflow
        sway drag (C_Dy ~ 1.0 on projected hull side area L * d_max), plus linear
        roll/pitch/yaw damping standing in for radiation damping.
        """
        wetted = mean_draft / self.max_draft if self.max_draft > 0 else 0.0
        hump = 1.0
        if hull_length and hull_length > 0.0 and hump_amp > 0.0:
            fr = abs(float(nu[0])) / np.sqrt(self.g * hull_length)
            hump = 1.0 + hump_amp * np.exp(-((fr - 0.48) / 0.22) ** 2)
        sway_coeff = (0.5 * self.rho * hull_length * self.max_draft * getattr(self, "n_hulls", 1)
                      if (hull_length and hull_length > 0.0 and self.max_draft > 0.0)
                      else 4.0 * drag_coeff)
        tau = np.zeros(6)
        tau[0] = -drag_coeff * hump * wetted * nu[0] * abs(nu[0])
        tau[1] = -sway_coeff * wetted * nu[1] * abs(nu[1])
        tau[3] = -rot_damping * wetted * nu[3]
        tau[4] = -rot_damping * wetted * nu[4]
        tau[5] = -rot_damping * wetted * nu[5]
        return tau
