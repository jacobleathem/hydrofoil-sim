import numpy as np

from hydrofoil.control.actuator import Actuator
from hydrofoil.control.pid import HydrofoilController


def test_actuator_settles_to_step_command():
    act = Actuator(dt=0.01, rate_limit_deg_s=720.0, tau=0.02, delay_steps=0)
    for _ in range(500):
        pos = act.update(10.0)
    assert abs(pos - 10.0) < 1e-3


def test_actuator_rate_limit_caps_initial_slew():
    act = Actuator(dt=0.01, rate_limit_deg_s=100.0, tau=1e-6, delay_steps=0)
    pos = act.update(90.0)   # one step, huge command
    assert pos <= 100.0 * 0.01 + 1e-9


def test_actuator_delay_holds_output_for_delay_steps():
    act = Actuator(dt=0.01, rate_limit_deg_s=1e6, tau=1e-6, delay_steps=3)
    outputs = [act.update(5.0) for _ in range(3)]
    assert all(o == 0.0 for o in outputs)   # command hasn't propagated through the delay yet
    assert abs(act.update(5.0) - 5.0) < 1e-3


def test_actuator_reset_clears_state():
    act = Actuator(dt=0.01, rate_limit_deg_s=720.0, tau=0.02, delay_steps=2)
    for _ in range(50):
        act.update(15.0)
    act.reset()
    assert act.update(0.0) == 0.0


def test_controller_output_is_actuator_filtered_not_raw_pid():
    gains = {
        'heave': {'Kp': 1.0, 'Ki': 0.0, 'Kd': 0.0},
        'pitch': {'Kp': 1.0, 'Ki': 0.0, 'Kd': 0.0},
        'roll':  {'Kp': 1.0, 'Ki': 0.0, 'Kd': 0.0},
    }
    # Slow actuator so the first-step output must be well short of the raw PID command.
    ctrl = HydrofoilController(gains, dt=0.01,
                                actuator_params={'rate_limit_deg_s': 5.0, 'tau': 0.5, 'delay_steps': 0})
    cmds = ctrl.update(z_ref=0.0, z_meas=10.0, phi_ref=0.0, phi_meas=0.0, theta_ref=0.0, theta_meas=0.0)
    # Raw PID would demand 10.0 deg (clamped to output range) immediately; actuator must lag it.
    assert 0.0 < cmds['delta_main'] < 1.0
