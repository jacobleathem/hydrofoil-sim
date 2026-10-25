"""
Static and dynamic buoyancy for a submerged hull.

For a fully submerged T-foil, buoyancy is dominated by the hull.
The foil struts contribute negligibly and are ignored here.
"""

import numpy as np


def buoyancy_force(
    displaced_volume: float,
    eta: np.ndarray,
    rho_water: float = 1025.0,
    g: float = 9.81,
) -> np.ndarray:
    """
    Generalized buoyancy force in body frame, correctly rotated for attitude.

    Buoyancy always acts in the NED -z direction (straight up), so it must be
    rotated into the body frame using the current vehicle attitude — exactly as
    gravity_force does for weight.  Without this rotation, the net hydrostatic
    force is non-zero whenever the vehicle pitches or rolls, producing a
    spurious surge/sway component.

    Assumes vehicle is always fully submerged (constant displaced volume).

    Parameters
    ----------
    displaced_volume : hull displaced volume [m³]
    eta              : (6,) [x, y, z, phi, theta, psi] — attitude used for rotation
    rho_water        : water density [kg/m³]
    g                : gravitational acceleration [m/s²]

    Returns
    -------
    tau_b : (6,) generalized force [X, Y, Z, K, M, N] in body frame
    """
    from hydrofoil.dynamics.kinematics import R_body_to_ned
    phi, theta, psi = eta[3], eta[4], eta[5]
    R = R_body_to_ned(phi, theta, psi)

    F_b = rho_water * g * displaced_volume
    f_ned = np.array([0.0, 0.0, -F_b])   # upward = -z in NED
    f_body = R.T @ f_ned

    tau = np.zeros(6)
    tau[:3] = f_body
    return tau


def gravity_force(mass: float, eta: np.ndarray, g: float = 9.81) -> np.ndarray:
    """
    Gravitational restoring force in body frame.

    Parameters
    ----------
    mass : vehicle mass [kg]
    eta  : (6,) [x, y, z, phi, theta, psi] — only angles used
    g    : gravitational acceleration [m/s²]

    Returns
    -------
    tau_g : (6,) generalized force in body frame
    """
    from hydrofoil.dynamics.kinematics import R_body_to_ned
    phi, theta, psi = eta[3], eta[4], eta[5]
    R = R_body_to_ned(phi, theta, psi)

    # Gravity in NED frame: [0, 0, m*g]
    f_ned = np.array([0.0, 0.0, mass * g])
    # Rotate to body frame
    f_body = R.T @ f_ned

    tau = np.zeros(6)
    tau[:3] = f_body
    return tau
