"""
Cascade PID controller for a fully submerged T-foil.

Three independent loops:
    heave  : altitude error → main foil flap command (delta_main)
    pitch  : pitch error    → elevator command        (delta_elev)
    roll   : roll error     → differential main flap  (delta_roll)

Each loop is a standard discrete-time PID with anti-windup clamping.
"""

import numpy as np

from hydrofoil.control.actuator import Actuator, DEFAULT_ACTUATOR_PARAMS


class PID:
    """
    Single-axis discrete-time PID with integrator anti-windup.

    Parameters
    ----------
    Kp, Ki, Kd  : gains
    dt           : sample time [s]
    output_min,
    output_max   : output clamp [same units as setpoint]
    """

    def __init__(
        self,
        Kp: float,
        Ki: float,
        Kd: float,
        dt: float,
        output_min: float = -np.inf,
        output_max: float = np.inf,
    ):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.output_min = output_min
        self.output_max = output_max

        self._integral = 0.0
        self._prev_measurement = None   # None = no derivative on the next call

    def reset(self):
        self._integral = 0.0
        self._prev_measurement = None

    def update(self, error: float, measurement: float) -> float:
        """
        Compute control output for one timestep.

        Parameters
        ----------
        error       : the caller's error signal (sign convention is the
                      caller's choice — this class doesn't assume one)
        measurement : the raw process value error was derived from, used
                      ONLY for the derivative term (see below)

        Derivative-on-measurement, not derivative-on-error: a sudden
        setpoint change (e.g. a pilot stick input) makes `error` jump
        discontinuously even though the plant hasn't moved, which would
        otherwise spike the D-term ("derivative kick") into a huge one-shot
        command on the very step the setpoint changes. Deriving from the
        measurement instead means the D-term only reacts to how fast the
        plant is actually moving, immune to setpoint jumps. Requires error
        to vary the same direction as measurement (true for every caller in
        this codebase's convention); reset() clears the reference so the
        first call after a restart also produces no spurious kick.

        Returns
        -------
        output : clamped control signal
        """
        if self._prev_measurement is None:
            derivative = 0.0
        else:
            derivative = (measurement - self._prev_measurement) / self.dt
        self._integral += error * self.dt

        output = self.Kp * error + self.Ki * self._integral + self.Kd * derivative

        # clamp and anti-windup: stop integrating when saturated
        clamped = np.clip(output, self.output_min, self.output_max)
        if output != clamped:
            self._integral -= error * self.dt   # undo last integration step

        self._prev_measurement = measurement
        return float(clamped)


class HydrofoilController:
    """
    Cascade PID for a T-foil with main foil + elevator.

    Outputs:
        delta_main  : symmetric main foil flap angle [deg]
        delta_elev  : elevator angle [deg]
        delta_roll  : differential main foil angle (left - right) [deg]

    Usage:
        ctrl = HydrofoilController(gains, dt=0.02)
        cmds = ctrl.update(z_ref, z_meas, phi_ref, phi_meas, theta_ref, theta_meas)
    """

    def __init__(self, gains: dict, dt: float, actuator_params: dict = None):
        """
        Parameters
        ----------
        gains           : dict with keys 'heave', 'pitch', 'roll', each containing
                          {'Kp': ..., 'Ki': ..., 'Kd': ...}
        dt              : sample time [s]
        actuator_params : kwargs for control.actuator.Actuator (rate_limit_deg_s,
                          tau, delay_steps), applied to every flap channel.
                          Defaults to DEFAULT_ACTUATOR_PARAMS (BLDC placeholder).
        """
        lim = 20.0   # ± 20 deg flap limit — adjust to your servo range

        self.heave_pid = PID(dt=dt, output_min=-lim, output_max=lim,
                             **gains['heave'])
        self.pitch_pid = PID(dt=dt, output_min=-lim, output_max=lim,
                             **gains['pitch'])
        self.roll_pid  = PID(dt=dt, output_min=-lim, output_max=lim,
                             **gains['roll'])

        actuator_params = actuator_params or DEFAULT_ACTUATOR_PARAMS
        self.main_actuator = Actuator(dt=dt, **actuator_params)
        self.elev_actuator = Actuator(dt=dt, **actuator_params)
        self.roll_actuator = Actuator(dt=dt, **actuator_params)

    def update(
        self,
        z_ref: float,     z_meas: float,
        phi_ref: float,   phi_meas: float,
        theta_ref: float, theta_meas: float,
    ) -> dict:
        """
        Compute foil deflection commands.

        Sign convention: positive delta → increased AoA → increased lift.
        NED frame: z positive downward, so altitude error = z_meas - z_ref
        (deeper = more positive = we want to rise = positive lift needed).

        Returns
        -------
        dict with 'delta_main', 'delta_elev', 'delta_roll' in [deg] — actual
        (actuator-filtered) flap angles, not raw PID output.
        """
        heave_err = z_meas - z_ref      # +ve: too deep → need more lift to rise
        pitch_err = theta_meas - theta_ref  # +ve: nose up → need upward rear force → –M_y → nose down
        roll_err  = phi_meas - phi_ref      # +ve: rolled right → need upward left force

        cmd_main = self.heave_pid.update(heave_err, z_meas)
        cmd_elev = self.pitch_pid.update(pitch_err, theta_meas)
        cmd_roll = self.roll_pid.update(roll_err, phi_meas)

        return {
            'delta_main': self.main_actuator.update(cmd_main),
            'delta_elev': self.elev_actuator.update(cmd_elev),
            'delta_roll': self.roll_actuator.update(cmd_roll),
        }

    def reset(self):
        self.heave_pid.reset()
        self.pitch_pid.reset()
        self.roll_pid.reset()
        self.main_actuator.reset()
        self.elev_actuator.reset()
        self.roll_actuator.reset()
