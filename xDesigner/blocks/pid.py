"""PIDController: PID with output saturation, anti-windup, and filtered derivative."""
from ..block import Block


class PIDController(Block):
    """Discrete-time PID controller.

    Ports
    -----
    in:  setpoint, measurement
    out: command  (clamped to [out_min, out_max])

    Features
    --------
    * Saturating output with conditional integration (anti-windup).
    * First-order low-pass filter on the derivative term to suppress
      noise (set ``deriv_filter_tau=0`` to disable).
    """

    def __init__(
        self,
        name: str,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
        out_min: float = -1.0,
        out_max: float = 1.0,
        deriv_filter_tau: float = 5e-3,
    ):
        super().__init__(name)
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.deriv_filter_tau = deriv_filter_tau

        self.add_input("setpoint")
        self.add_input("measurement")
        self.add_output("command")

        self._integral = 0.0
        self._prev_error = 0.0
        self._deriv = 0.0

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = 0.0
        self._deriv = 0.0

    def step(self, t: float, dt: float) -> None:
        err = self.inputs["setpoint"].value - self.inputs["measurement"].value

        # filtered derivative
        if dt > 0:
            raw_deriv = (err - self._prev_error) / dt
            if self.deriv_filter_tau > 0:
                alpha = dt / (self.deriv_filter_tau + dt)
                self._deriv += alpha * (raw_deriv - self._deriv)
            else:
                self._deriv = raw_deriv

        # provisional unclamped output (anti-windup decision)
        u_unsat = self.kp * err + self.ki * self._integral + self.kd * self._deriv
        u_sat = max(self.out_min, min(self.out_max, u_unsat))

        # only integrate when not pushing further into saturation
        saturated = (u_unsat != u_sat)
        if not (saturated and (err * (u_unsat - u_sat) > 0.0)):
            self._integral += err * dt

        # final output (with updated integral)
        u = self.kp * err + self.ki * self._integral + self.kd * self._deriv
        u = max(self.out_min, min(self.out_max, u))
        self.outputs["command"].value = u

        self._prev_error = err
