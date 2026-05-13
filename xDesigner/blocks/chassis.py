"""Chassis: longitudinal vehicle chassis (rolling resistance + aero drag).

Reads velocity, outputs the total resistive force opposing motion plus the
running position (integral of velocity). The reaction force is what the
``DCMotor`` block consumes through its ``F_resist`` input.
"""
import math
from ..block import Block


class Chassis(Block):
    """One-dimensional vehicle chassis model.

    F_resist(v) = mass * g * Crr * sign(v) + 0.5 * rho * Cd * A * v * |v|

    The rolling-resistance term uses a soft sign (tanh) so the solver
    doesn't get jittery near v ≈ 0.

    Parameters
    ----------
    mass : vehicle mass (kg)
    Crr  : rolling resistance coefficient (typical 0.01–0.03 for small wheels)
    Cd   : drag coefficient
    frontal_area : projected frontal area (m^2)
    air_density  : kg/m^3 (1.225 at sea level, 20 °C)
    g    : gravitational acceleration (m/s^2)

    Ports
    -----
    in:  velocity (m/s)
    out: F_resist (N), position (m)
    """

    def __init__(
        self,
        name: str,
        mass: float = 1.0,
        Crr: float = 0.015,
        Cd: float = 0.5,
        frontal_area: float = 0.01,
        air_density: float = 1.225,
        g: float = 9.81,
    ):
        super().__init__(name)
        self.mass = mass
        self.Crr = Crr
        self.Cd = Cd
        self.frontal_area = frontal_area
        self.air_density = air_density
        self.g = g

        self.add_input("velocity")
        self.add_output("F_resist")
        self.add_output("position")

        self.x = 0.0

    def reset(self) -> None:
        self.x = 0.0

    def step(self, t: float, dt: float) -> None:
        v = self.inputs["velocity"].value

        # Soft sign avoids chattering near v=0 for the rolling-resistance term.
        # tanh(v / v_eps) ≈ sign(v) for |v| >> v_eps and is smooth around 0.
        v_eps = 1e-3
        soft_sign = math.tanh(v / v_eps)

        F_roll = self.Crr * self.mass * self.g * soft_sign
        F_drag = 0.5 * self.air_density * self.Cd * self.frontal_area * v * abs(v)

        self.outputs["F_resist"].value = F_roll + F_drag

        self.x += v * dt
        self.outputs["position"].value = self.x
