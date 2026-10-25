"""
Unit tests for physics models.

Run with: python -m pytest tests/ -v
"""

import numpy as np
import pytest
from hydrofoil.dynamics.kinematics import J, R_body_to_ned
from hydrofoil.dynamics.rigid_body import RigidBody, skew
from hydrofoil.forces.foil import Foil, FoilGeometry, FoilTables
from hydrofoil.forces.buoyancy import buoyancy_force, gravity_force

POLAR = FoilTables.naca0009()


# --- NACA 0009 polar ---

def test_naca0009_loads():
    assert len(POLAR.aoa_deg) > 100  # 119 rows in the XFLR5 file

def test_naca0009_symmetric():
    """Symmetric foil: CL(+α) = -CL(-α), CD(+α) = CD(-α)."""
    for a in [1.0, 3.0, 5.0, 8.0]:
        assert abs(POLAR.cl(a) + POLAR.cl(-a)) < 1e-4, f"CL asymmetry at {a} deg"
        assert abs(POLAR.cd(a) - POLAR.cd(-a)) < 1e-4, f"CD asymmetry at {a} deg"

def test_naca0009_zero_lift_at_zero_aoa():
    assert abs(POLAR.cl(0.0)) < 1e-6

def test_naca0009_cl_slope():
    # Thin-airfoil theory: 2π/rad = 0.1097/deg. NACA 0009 should be close.
    slope = POLAR.cl_slope_per_deg
    assert 0.10 < slope < 0.12, f"CL slope {slope:.4f}/deg outside expected range"

def test_naca0009_stall():
    # CL should peak near 14-15 deg and not extrapolate past table bounds
    cl_at_20 = POLAR.cl(20.0)   # beyond table — should clamp to last entry
    assert cl_at_20 < 1.5, "Clamping past stall should not produce unrealistically high CL"


# --- Kinematics ---

def test_rotation_matrix_orthogonal():
    R = R_body_to_ned(0.1, -0.05, 0.3)
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-10)
    assert np.allclose(np.linalg.det(R), 1.0, atol=1e-10)


def test_J_at_zero_angles_is_identity():
    eta = np.zeros(6)
    Jm = J(eta)
    assert np.allclose(Jm, np.eye(6), atol=1e-10)


# --- Rigid body ---

def test_skew_antisymmetric():
    v = np.array([1.0, 2.0, 3.0])
    S = skew(v)
    assert np.allclose(S + S.T, 0)


def test_rigid_body_M_symmetric():
    rb = RigidBody(mass=25.0, I_b=np.diag([2.0, 5.0, 5.0]))
    assert np.allclose(rb.M, rb.M.T)


def test_rigid_body_M_positive_definite():
    rb = RigidBody(mass=25.0, I_b=np.diag([2.0, 5.0, 5.0]))
    eigvals = np.linalg.eigvalsh(rb.M)
    assert np.all(eigvals > 0)


def test_rigid_body_zero_force_zero_accel():
    rb = RigidBody(mass=25.0, I_b=np.diag([2.0, 5.0, 5.0]))
    nu = np.zeros(6)
    nu_dot = rb.nu_dot(nu, np.zeros(6))
    assert np.allclose(nu_dot, 0)


# --- Foil forces ---

def test_foil_zero_speed_returns_zero():
    foil = Foil(FoilGeometry(0.15, 0.6), POLAR, r_attach=np.array([0, 0, 0.5]))
    nu = np.zeros(6)
    tau = foil.generalized_force(nu)
    assert np.allclose(tau, 0)


def test_foil_produces_upward_force_at_positive_aoa():
    foil = Foil(FoilGeometry(0.15, 0.6), POLAR, r_attach=np.array([0, 0, 0.5]))
    # Positive flap delta → positive AoA → upward lift (negative NED z force)
    nu = np.array([4.0, 0, 0, 0, 0, 0])
    tau = foil.generalized_force(nu, delta=5.0)
    assert tau[2] < 0, f"Expected upward (negative z) lift, got Z={tau[2]:.3f}"

def test_foil_sinking_vehicle_produces_restoring_lift():
    foil = Foil(FoilGeometry(0.15, 0.6), POLAR, r_attach=np.array([0, 0, 0.5]))
    # Vehicle sinking (w > 0 in NED) → positive AoA → upward lift → restoring
    nu = np.array([4.0, 0, 0.5, 0, 0, 0])   # 0.5 m/s heave downward
    tau = foil.generalized_force(nu, delta=0.0)
    assert tau[2] < 0, f"Sinking should produce upward lift, got Z={tau[2]:.3f}"


def test_foil_drag_opposes_motion():
    foil = Foil(FoilGeometry(0.15, 0.6), POLAR, r_attach=np.array([0, 0, 0.5]))
    nu = np.array([4.0, 0, 0, 0, 0, 0])
    tau = foil.generalized_force(nu)
    # Drag should oppose forward motion = negative X force
    assert tau[0] < 0, f"Expected negative X drag, got X={tau[0]:.3f}"


# --- Buoyancy / gravity ---

def test_buoyancy_upward():
    tau = buoyancy_force(displaced_volume=0.025, eta=np.zeros(6))
    assert tau[2] < 0   # upward = negative NED z


def test_gravity_downward_at_zero_angle():
    eta = np.zeros(6)
    tau = gravity_force(mass=25.0, eta=eta)
    assert tau[2] > 0   # downward = positive NED z


def test_buoyancy_gravity_balance_at_neutral():
    m = 25.0
    rho = 1025.0
    g = 9.81
    V = m / rho
    eta = np.zeros(6)
    tau_b = buoyancy_force(V, eta, rho, g)
    tau_g = gravity_force(m, eta, g)
    net_z = tau_b[2] + tau_g[2]
    assert abs(net_z) < 1e-6, f"Neutral buoyancy should be zero net, got {net_z:.6f}"
