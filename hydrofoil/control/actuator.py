"""
Actuator dynamics for a single flap channel: command latency, rate limit, and
first-order lag between a commanded angle and the flap's actual position.

Placeholder parameters model a MyActuator RMD-X8-P20-120-class direct-drive
BLDC actuator (CAN-bus position control) rather than a hobby RC servo — fast
rate limit, small lag, latency dominated by CAN bus + the actuator's internal
control loop. Replace DEFAULT_ACTUATOR_PARAMS with real datasheet numbers
(rated speed, torque-limited slew rate, CAN update rate) once measured.
"""

from collections import deque

import numpy as np

DEFAULT_ACTUATOR_PARAMS = {
    "rate_limit_deg_s": 720.0,  # placeholder BLDC slew rate
    "tau": 0.02,                # first-order lag time constant [s]
    "delay_steps": 2,           # pure command latency, in units of dt (CAN + loop)
}


class Actuator:
    """
    Models one flap actuator: pure delay -> rate-limited target -> first-order lag.

    Parameters
    ----------
    dt               : sample time [s], must match the controller's dt
    rate_limit_deg_s : max slew rate [deg/s]
    tau              : first-order lag time constant [s]
    delay_steps      : number of dt-steps of pure command latency
    """

    def __init__(self, dt: float, rate_limit_deg_s: float, tau: float, delay_steps: int):
        self.dt = dt
        self.rate_limit_deg_s = rate_limit_deg_s
        self.tau = tau
        self.delay_steps = delay_steps

        self._position = 0.0
        self._delay_buffer = deque([0.0] * delay_steps, maxlen=delay_steps) if delay_steps > 0 else None

    def reset(self):
        self._position = 0.0
        if self._delay_buffer is not None:
            self._delay_buffer = deque([0.0] * self.delay_steps, maxlen=self.delay_steps)

    def update(self, command: float) -> float:
        """Advance actuator state by one dt and return the resulting flap angle [deg]."""
        if self._delay_buffer is not None:
            delayed_command = self._delay_buffer[0]
            self._delay_buffer.append(command)
            command = delayed_command

        max_step = self.rate_limit_deg_s * self.dt
        target = self._position + np.clip(command - self._position, -max_step, max_step)

        alpha = self.dt / (self.tau + self.dt)
        self._position += alpha * (target - self._position)

        return self._position
