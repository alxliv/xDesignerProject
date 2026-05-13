"""Steering: maps a normalised command in [-1, +1] to a steering angle.

Models a hobby-servo-driven steering linkage:
    * saturation at ``delta_max``;
    * first-order actuator lag with time constant ``tau``;
    * optional centring bias so command = 0 corresponds to delta = 0.

The output ``delta`` is consumed by ``Chassis2D``.
"""
import math
from ..block import Block


class Steering(Block):
    """Steering actuator.

    Parameters
    ----------
    delta_max : maximum steering angle (rad). Defaults to ~30°.
    tau       : first-order time constant of the servo (s). 0 disables lag.

    Ports
    -----
    in:  command  (dimensionless, expected in [-1, +1])
    out: delta    (rad, signed, clipped to [-delta_max, delta_max])
    """

    def __init__(
        self,
        name: str,
        delta_max: float = math.radians(30.0),
        tau: float = 0.05,
    ):
        super().__init__(name)
        if delta_max <= 0:
            raise ValueError("delta_max must be > 0")
        if tau < 0:
            raise ValueError("tau must be >= 0")
        self.delta_max = delta_max
        self.tau = tau

        self.add_input("command")
        self.add_output("delta")

        self.delta = 0.0

    def reset(self) -> None:
        self.delta = 0.0

    def step(self, t: float, dt: float) -> None:
        cmd = self.inputs["command"].value
        # saturate command
        if cmd > 1.0:
            cmd = 1.0
        elif cmd < -1.0:
            cmd = -1.0
        target = cmd * self.delta_max

        if self.tau > 0 and dt > 0:
            alpha = dt / (self.tau + dt)
            self.delta += alpha * (target - self.delta)
        else:
            self.delta = target

        self.outputs["delta"].value = self.delta
