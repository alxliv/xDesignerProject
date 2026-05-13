"""Chassis2D: kinematic-bicycle chassis with x-y-θ pose.

Coexists with the 1D ``Chassis`` block from v0.1. The motor still owns
the longitudinal mechanical state (its ``velocity`` output is body-axis
longitudinal speed in m/s); this block adds:

* yaw dynamics from steering angle via the kinematic bicycle:
    ψ̇  = v · tan(δ) / L
    θ̇  = ψ̇
    ẋ  = v · cos(θ)
    ẏ  = v · sin(θ)

* The same rolling-resistance + aerodynamic-drag ``F_resist`` as the 1D
  chassis, so the wiring to ``DCMotor.F_resist`` is identical.

This is a *kinematic* model: no tire slip, no lateral grip limit. A
future ``Chassis2D_dynamic`` block can replace this when slip matters.
"""
import math
from ..block import Block


class Chassis2D(Block):
    """Kinematic-bicycle 2D chassis.

    Parameters
    ----------
    mass         : vehicle mass (kg)
    wheelbase    : front-to-rear axle distance L (m)
    Crr          : rolling-resistance coefficient
    Cd           : aerodynamic drag coefficient
    frontal_area : projected frontal area (m^2)
    air_density  : kg/m^3
    g            : gravitational acceleration (m/s^2)
    x0, y0, theta0 : initial pose

    Ports
    -----
    in:  velocity (m/s, body-axis longitudinal — typically from DCMotor)
         delta    (rad, steering angle — typically from Steering)
    out: F_resist (N, longitudinal resistance feeding back to DCMotor)
         x, y     (m, world frame)
         theta    (rad, heading, wrapped to (-pi, pi])
         yaw_rate (rad/s)
    """

    def __init__(
        self,
        name: str,
        mass: float = 1.0,
        wheelbase: float = 0.15,
        Crr: float = 0.015,
        Cd: float = 0.6,
        frontal_area: float = 0.012,
        air_density: float = 1.225,
        g: float = 9.81,
        x0: float = 0.0,
        y0: float = 0.0,
        theta0: float = 0.0,
    ):
        super().__init__(name)
        if wheelbase <= 0:
            raise ValueError("wheelbase must be > 0")
        if mass <= 0:
            raise ValueError("mass must be > 0")
        self.mass = mass
        self.wheelbase = wheelbase
        self.Crr = Crr
        self.Cd = Cd
        self.frontal_area = frontal_area
        self.air_density = air_density
        self.g = g
        self.x0 = x0
        self.y0 = y0
        self.theta0 = theta0

        self.add_input("velocity")
        self.add_input("delta")
        self.add_output("F_resist")
        self.add_output("x")
        self.add_output("y")
        self.add_output("theta")
        self.add_output("yaw_rate")

        self.x = x0
        self.y = y0
        self.theta = theta0

    def reset(self) -> None:
        self.x = self.x0
        self.y = self.y0
        self.theta = self.theta0

    def step(self, t: float, dt: float) -> None:
        v = self.inputs["velocity"].value
        delta = self.inputs["delta"].value

        # Resistive force (identical formula to 1D Chassis).
        v_eps = 1e-3
        soft_sign = math.tanh(v / v_eps)
        F_roll = self.Crr * self.mass * self.g * soft_sign
        F_drag = 0.5 * self.air_density * self.Cd * self.frontal_area * v * abs(v)
        F_resist = F_roll + F_drag

        # Kinematic bicycle update (forward Euler).
        yaw_rate = v * math.tan(delta) / self.wheelbase
        # use mid-step heading for better accuracy at fixed dt
        theta_mid = self.theta + 0.5 * yaw_rate * dt
        self.x += v * math.cos(theta_mid) * dt
        self.y += v * math.sin(theta_mid) * dt
        self.theta += yaw_rate * dt
        # wrap to (-pi, pi]
        if self.theta > math.pi:
            self.theta -= 2.0 * math.pi
        elif self.theta <= -math.pi:
            self.theta += 2.0 * math.pi

        self.outputs["F_resist"].value = F_resist
        self.outputs["x"].value = self.x
        self.outputs["y"].value = self.y
        self.outputs["theta"].value = self.theta
        self.outputs["yaw_rate"].value = yaw_rate
