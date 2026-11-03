"""
Side force and drag on the vertical struts (masts) that carry the foils.

The boat has two masts: the front one carries the split main foil and the
motors, the rear one carries the elevator. Each runs from its foil up to the
hull. Below the water surface a mast is a vertical lifting surface: when the
boat slips sideways, it meets the flow at a sideslip angle and makes "lift"
sideways, opposing the slip. That side force is what makes a bank-to-turn
boat actually turn:

  - The rear mast sits aft of the CoG, so its side force swings the bow into
    the direction of travel (weathercock stability). Without it the heading
    is nearly free: banking only makes the boat slide sideways.
  - Spinning in yaw sweeps both masts sideways through the water, which damps
    the yaw rate.
  - Both masts hang below the CoG, so their side force also rolls the boat.

Same strip model as forces/foil.py (one strip per surface, section polar,
finite-span correction, induced drag), with the angle of attack taken in the
horizontal (x-y) body plane instead of the vertical (x-z) one. Only the part
of the mast below the local water surface counts, so the force fades out as
the boat climbs and the wetted length shrinks. Ventilation of the
surface-piercing tip is not modelled.
"""

from __future__ import annotations

import numpy as np

from hydrofoil.dynamics.kinematics import R_body_to_ned
from hydrofoil.forces.foil import FoilTables

# Below this wetted length [m] a mast is treated as out of the water.
_MIN_WETTED = 1e-3


class Strut:
    """
    One vertical surface-piercing strut.

    Parameters
    ----------
    chord   : strut chord (fore-aft length of its section) [m]
    x       : fore-aft position of the strut in the body frame [m], +forward
    z_foot  : body-frame z of the strut's lower end (the foil it carries) [m],
              +down (NED)
    z_top   : body-frame z of the strut's upper end (the hull bottom) [m]
    tables  : section polar; a symmetric section (NACA 0009) suits a strut
    """

    def __init__(self, chord: float, x: float, z_foot: float, z_top: float,
                 tables: FoilTables, rho: float = 1025.0, oswald_efficiency: float = 0.80):
        if z_top >= z_foot:
            raise ValueError("strut top must be above its foot (z_top < z_foot in NED)")
        self.chord = chord
        self.x = x
        self.z_foot = z_foot
        self.z_top = z_top
        self.length = z_foot - z_top
        self.tables = tables
        self.rho = rho
        self.oswald_efficiency = oswald_efficiency

    def wetted_length(self, eta: np.ndarray, water_surface_z: float) -> float:
        """
        Length of strut below the local water surface [m], from the foot up.
        Uses the foot's world depth and the strut's vertical extent in the
        world frame (it leans with roll and pitch).
        """
        R = R_body_to_ned(eta[3], eta[4], eta[5])
        foot_depth = eta[2] + (R @ np.array([self.x, 0.0, self.z_foot]))[2] - water_surface_z
        # World-frame vertical rise per metre of strut (cos of its tilt).
        rise = R[2, 2]
        if rise <= 1e-3 or foot_depth <= 0.0:
            return 0.0
        return float(np.clip(foot_depth / rise, 0.0, self.length))

    def added_mass_sway(self, wetted_length: float) -> float:
        """Sway added mass [kg] of a flat plate of this chord moving broadside: rho*pi*c^2/4 per metre."""
        return self.rho * np.pi * self.chord**2 / 4.0 * wetted_length

    def generalized_force(self, nu: np.ndarray, eta: np.ndarray,
                          water_surface_z: float = 0.0) -> np.ndarray:
        """
        6-DOF generalized force [X, Y, Z, K, M, N] in the body frame from
        this strut's wetted part. nu should be the velocity relative to the
        water (waves included), as for Foil.generalized_force.
        """
        wetted = self.wetted_length(eta, water_surface_z)
        if wetted < _MIN_WETTED:
            return np.zeros(6)

        # Centre of pressure: middle of the wetted part, measured up the
        # strut from its foot.
        r_cp = np.array([self.x, 0.0, self.z_foot - wetted / 2.0])

        V = nu[:3] + np.cross(nu[3:], r_cp)
        # A strut only sees flow across its x-y section; vertical flow runs
        # along the span and makes no side force.
        V_xy = np.array([V[0], V[1], 0.0])
        speed = np.linalg.norm(V_xy)
        if speed < 0.01:
            return np.zeros(6)
        V_hat = V_xy / speed

        # Sideslip angle at the strut. Moving to starboard (v > 0) means the
        # water arrives from starboard: positive beta, force to port (-y).
        # Fold reverse flow (u < 0) onto the forward polar; the sign flip on
        # side_dir below keeps the force opposing the slip.
        beta_deg = np.degrees(np.arctan2(V[1], abs(V[0])))

        # One end is closed by the foil, the other by the free surface; both
        # act roughly as end plates, so take the wetted length as the full
        # effective span.
        aspect_ratio = wetted / self.chord
        CL = self.tables.cl(beta_deg) * aspect_ratio / (aspect_ratio + 2.0)
        CD = self.tables.cd(beta_deg) + CL**2 / (np.pi * aspect_ratio * self.oswald_efficiency)

        q = 0.5 * self.rho * speed**2
        A = self.chord * wetted

        # Side "lift" lies in the x-y plane, perpendicular to the flow, and
        # opposes the slip: rotate V_hat by -90 deg about body z, i.e.
        # (vx, vy) -> (vy, -vx), then take sign from CL (which carries beta).
        side_dir = np.array([V_hat[1], -V_hat[0], 0.0])
        force = q * A * (CL * side_dir * np.sign(V_hat[0] or 1.0) - CD * V_hat)
        moment = np.cross(r_cp, force)

        tau = np.zeros(6)
        tau[:3] = force
        tau[3:] = moment
        return tau
