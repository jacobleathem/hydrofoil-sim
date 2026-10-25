"""
Kinematic transformation matrix J(eta) for 6-DOF marine vehicle.

Maps body-frame velocities (nu) to world-frame rates (eta_dot):
    eta_dot = J(eta) @ nu

State vectors follow Fossen (2011) notation:
    eta = [x, y, z, phi, theta, psi]   (NED position + ZYX Euler angles)
    nu  = [u, v, w, p, q, r]           (body-frame linear + angular velocity)
"""

import numpy as np


def Rx(phi: float) -> np.ndarray:
    c, s = np.cos(phi), np.sin(phi)
    return np.array([[1, 0, 0],
                     [0, c, -s],
                     [0, s,  c]])


def Ry(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[ c, 0, s],
                     [ 0, 1, 0],
                     [-s, 0, c]])


def Rz(psi: float) -> np.ndarray:
    c, s = np.cos(psi), np.sin(psi)
    return np.array([[c, -s, 0],
                     [s,  c, 0],
                     [0,  0, 1]])


def R_body_to_ned(phi: float, theta: float, psi: float) -> np.ndarray:
    """ZYX rotation: world = Rz(psi) @ Ry(theta) @ Rx(phi) @ body."""
    return Rz(psi) @ Ry(theta) @ Rx(phi)


def T_euler(phi: float, theta: float) -> np.ndarray:
    """
    Angular velocity transformation from body rates (p, q, r) to
    Euler angle rates (phi_dot, theta_dot, psi_dot).

    Singular at theta = ±90 deg (gimbal lock). For hydrofoils operating
    near level flight this is not a practical concern.
    """
    ct = np.cos(theta)
    if abs(ct) < 1e-6:
        raise ValueError(f"Euler singularity: theta={np.degrees(theta):.1f} deg")
    st = np.sin(theta)
    cp, sp = np.cos(phi), np.sin(phi)
    tt = st / ct
    return np.array([[1, sp * tt, cp * tt],
                     [0, cp,      -sp     ],
                     [0, sp / ct, cp / ct]])


def J(eta: np.ndarray) -> np.ndarray:
    """
    Full 6x6 kinematic Jacobian.

    Parameters
    ----------
    eta : (6,) array — [x, y, z, phi, theta, psi]

    Returns
    -------
    J : (6, 6) array such that eta_dot = J @ nu
    """
    phi, theta, psi = eta[3], eta[4], eta[5]
    R = R_body_to_ned(phi, theta, psi)
    T = T_euler(phi, theta)
    J_out = np.zeros((6, 6))
    J_out[:3, :3] = R
    J_out[3:, 3:] = T
    return J_out
