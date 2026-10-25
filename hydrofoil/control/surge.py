"""
Forward-speed (surge) controller: closes the loop from a target cruise speed
to a commanded thrust, so the vehicle actually accelerates from rest instead
of having its speed force-fed as a constant.
"""

from hydrofoil.control.pid import PID


class SurgeController:
    """
    Single PID loop: speed error -> thrust command [N], clamped to
    [-max_thrust, +max_thrust].
    """

    def __init__(self, Kp: float, Ki: float, Kd: float, dt: float, max_thrust: float):
        self.pid = PID(Kp=Kp, Ki=Ki, Kd=Kd, dt=dt,
                        output_min=-max_thrust, output_max=max_thrust)

    def update(self, speed_ref: float, speed_meas: float) -> float:
        error = speed_ref - speed_meas
        return self.pid.update(error, speed_meas)

    def reset(self):
        self.pid.reset()
