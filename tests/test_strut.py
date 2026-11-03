import numpy as np
import pytest

from hydrofoil.dashboard import DEFAULT_PARAMS
from hydrofoil.dynamics.rigid_body import RigidBody
from hydrofoil.forces.foil import FoilTables
from hydrofoil.forces.strut import Strut
from hydrofoil.vehicle import VehiclePhysics
from hydrofoil.waves.airy import CalmWater

POLAR = FoilTables.naca0009()


def _strut(x=0.0):
    # 1.1 m mast from a foil 0.5 m below the CoG up to a hull 0.6 m above it.
    return Strut(0.10, x=x, z_foot=0.5, z_top=-0.6, tables=POLAR)


def _eta(z=0.05, phi=0.0, theta=0.0):
    return np.array([0.0, 0.0, z, phi, theta, 0.0])


def _nu(u=6.0, v=0.0, r=0.0):
    return np.array([u, v, 0.0, 0.0, 0.0, r])


# ── Strut ────────────────────────────────────────────────────────────────

def test_wetted_length_follows_depth():
    s = _strut()
    assert s.wetted_length(_eta(z=0.05), 0.0) == pytest.approx(0.55)
    assert s.wetted_length(_eta(z=-0.5), 0.0) == 0.0          # foil out of the water
    assert s.wetted_length(_eta(z=1.0), 0.0) == pytest.approx(1.1)   # capped at its length
    # A wave crest over the boat wets more of it.
    assert s.wetted_length(_eta(z=0.05), -0.1) == pytest.approx(0.65)


def test_no_side_force_without_sideslip():
    tau = _strut().generalized_force(_nu(v=0.0), _eta())
    assert tau[1] == pytest.approx(0.0, abs=1e-9)
    assert tau[0] < 0.0          # profile drag only


def test_side_force_opposes_sideslip_both_ways():
    s = _strut()
    right = s.generalized_force(_nu(v=0.3), _eta())
    left = s.generalized_force(_nu(v=-0.3), _eta())
    assert right[1] < 0.0 < left[1]
    assert right[1] == pytest.approx(-left[1])


def test_side_force_opposes_sideslip_when_going_backwards():
    tau = _strut().generalized_force(_nu(u=-2.0, v=0.3), _eta())
    assert tau[1] < 0.0


def test_no_force_when_out_of_the_water():
    assert np.all(_strut().generalized_force(_nu(v=0.3), _eta(z=-0.5)) == 0.0)


def test_rear_strut_turns_the_bow_into_the_flow():
    # Slipping to starboard, the rear mast's side force swings the bow to
    # starboard (positive yaw moment N): weathercock stability.
    tau = _strut(x=-1.2).generalized_force(_nu(v=0.3), _eta())
    assert tau[5] > 0.0


def test_yaw_rate_is_damped():
    # Yawing right with no sideslip: the rear mast is swept to port, and its
    # force pushes back against the yaw.
    tau = _strut(x=-1.2).generalized_force(_nu(r=0.3), _eta())
    assert tau[5] < 0.0


def test_side_force_shrinks_with_wetted_length():
    s = _strut()
    deep = s.generalized_force(_nu(v=0.3), _eta(z=0.3))
    shallow = s.generalized_force(_nu(v=0.3), _eta(z=-0.2))
    assert abs(shallow[1]) < abs(deep[1])


def test_strut_rejects_top_below_foot():
    with pytest.raises(ValueError):
        Strut(0.1, x=0.0, z_foot=-0.6, z_top=0.5, tables=POLAR)


# ── RigidBody sway/yaw added mass ────────────────────────────────────────

def test_rigid_body_sway_yaw_added_mass():
    rb = RigidBody(10.0, np.diag([1.0, 2.0, 3.0]), added_mass_sway=4.0, added_inertia_yaw=0.5)
    assert rb.M[1, 1] == pytest.approx(14.0)
    assert rb.M[5, 5] == pytest.approx(3.5)
    assert rb.M[0, 0] == pytest.approx(10.0)


# ── Whole vehicle ────────────────────────────────────────────────────────

def _fly_banked(bank_deg, t_end=40.0, **overrides):
    params = dict(DEFAULT_PARAMS, target_speed=6.0, **overrides)
    vp = VehiclePhysics(params)
    state, t, water = vp.rest_state(), 0.0, CalmWater()
    while t < t_end:
        params["phi_ref"] = bank_deg if t >= 20.0 else 0.0
        state, *_ = vp.step(state, t, water)
        t += params["dt"]
    return state


def test_banked_boat_turns_with_little_sideslip():
    state = _fly_banked(15.0)
    u, v, r = state[6], state[7], state[11]
    assert np.degrees(state[3]) == pytest.approx(15.0, abs=1.0)   # holds the commanded bank
    assert r > 0.03                                               # turning right
    assert abs(np.degrees(np.arctan2(v, u))) < 2.0                # bow follows the track


def test_without_struts_the_boat_slides_instead():
    state = _fly_banked(15.0, struts_enabled=False)
    assert abs(np.degrees(np.arctan2(state[7], state[6]))) > 4.0


def test_struts_enabled_adds_sway_and_yaw_added_mass():
    with_s = VehiclePhysics(dict(DEFAULT_PARAMS)).rigid_body.M
    without = VehiclePhysics(dict(DEFAULT_PARAMS, struts_enabled=False)).rigid_body.M
    assert with_s[1, 1] > without[1, 1] == DEFAULT_PARAMS["mass"]
    assert with_s[5, 5] > without[5, 5] == DEFAULT_PARAMS["izz"]


# ── Stall, stiffness and the retuned controller ──────────────────────────

def test_section_is_symmetric_and_stalls_past_the_table():
    s = _strut()
    cl_pos, cd_pos = s.section_coefficients(8.0)
    cl_neg, cd_neg = s.section_coefficients(-8.0)
    assert cl_neg == pytest.approx(-cl_pos) and cd_neg == pytest.approx(cd_pos)
    # Way past stall: much less lift than the table's peak, lots of drag,
    # and no lift at all broadside.
    cl_45, cd_45 = s.section_coefficients(45.0)
    assert cl_45 < POLAR.CL.max() and cd_45 > 0.9
    assert s.section_coefficients(90.0)[0] == pytest.approx(0.0, abs=1e-9)


def test_stalled_strut_does_not_push_the_boat_forward():
    # 50 deg of sideslip: before the stall model this gave a forward
    # (propulsive) force and a hard-rolling boat could scull to 15 m/s.
    u, v = 6.0 * np.cos(np.radians(50)), 6.0 * np.sin(np.radians(50))
    tau = _strut().generalized_force(_nu(u=u, v=v), _eta())
    V_hat = np.array([u, v, 0.0]) / 6.0
    assert tau[:3] @ V_hat < 0.0          # never adds energy along the flow


def test_substeps_grow_with_speed_and_wetted_length():
    vp = VehiclePhysics(dict(DEFAULT_PARAMS))
    cruise = np.concatenate([_eta(z=0.05), _nu(u=6.0)])
    deep_fast = np.concatenate([_eta(z=0.55), _nu(u=12.0)])
    assert vp._substeps(cruise, 0.0, 0.01) <= 2
    assert vp._substeps(deep_fast, 0.0, 0.01) > vp._substeps(cruise, 0.0, 0.01)
    assert VehiclePhysics(dict(DEFAULT_PARAMS, struts_enabled=False))._substeps(deep_fast, 0.0, 0.01) == 1


def test_fast_with_hull_low_stays_bounded():
    # Regression: at ~8 m/s with the hull low and both masts fully wet, a
    # single dt = 0.01 RK4 step went unstable in yaw and the state blew up
    # to the clamps.
    params = dict(DEFAULT_PARAMS, target_speed=9.0, flying_height=-0.1, phi_ref=15.0)
    vp = VehiclePhysics(params)
    state = np.concatenate([_eta(z=0.6), _nu(u=8.5, v=0.3, r=0.3)])
    water = CalmWater()
    for i in range(1000):
        state, *_ = vp.step(state, i * 0.01, water)
        assert abs(state[11]) < 2.0 and state[6] < 11.0


def test_integrals_are_held_while_the_hull_is_in_the_water():
    # Sitting at rest below flying height, the heave error never goes away.
    # The integral must not build on it (the proportional term alone may
    # still hold the flap at its limit, which helps takeoff and unwinds at
    # once).
    params = dict(DEFAULT_PARAMS, target_speed=0.0)
    vp = VehiclePhysics(params)
    state, water = vp.rest_state(), CalmWater()
    for i in range(3000):
        state, *_ = vp.step(state, i * 0.01, water)
    assert vp.controller.heave_pid._integral == 0.0
    assert vp.controller.pitch_pid._integral == 0.0


def test_height_step_settles_quickly():
    params = dict(DEFAULT_PARAMS, target_speed=6.0)
    vp = VehiclePhysics(params)
    state, t, water = vp.rest_state(), 0.0, CalmWater()
    hull_height = []
    while t < 25.0:
        params["flying_height"] = 0.55 if t < 15.0 else 0.35
        state, *_ = vp.step(state, t, water)
        t += params["dt"]
        if t > 17.0:
            hull_height.append(params["hull_offset"] - state[2])
    assert np.max(np.abs(np.array(hull_height) - 0.35)) < 0.03
