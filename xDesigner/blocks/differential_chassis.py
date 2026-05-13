"""DifferentialChassis: 2D chassis for a skid-steer / differential-drive rover.

Two independently driven wheels, separated by ``track_width`` T. Each
wheel's longitudinal surface speed is an input; the chassis derives:

    v        = (v_L + v_R) / 2
    yaw_rate = (v_R - v_L) / T

and integrates the kinematic pose (x, y, θ). The longitudinal resistance
is the same rolling + drag formula as ``Chassis2D``, split evenly between
the two wheels. An additional **scrub** term — proportional to |yaw_rate|
— is added to each wheel's resistance, modelling the lateral friction
that skid-steer vehicles pay when turning. That makes "uses least power
wins" a meaningful objective: tighter turns cost more energy.
"""
import math
from ..block import Block


class DifferentialChassis(Block):
    """Differential-drive 2D chassis.

    Parameters
    ----------
    mass         : vehicle mass (kg)
    track_width  : lateral distance between the two driven wheels (m)
    Crr          : rolling-resistance coefficient
    Cd           : aerodynamic drag coefficient
    frontal_area : projected frontal area (m^2)
    scrub_coeff  : skid-friction coefficient — extra rolling resistance
                   each wheel pays per unit |yaw_rate|, expressed as a
                   fraction of mg per (rad/s). Set 0 to disable.
    air_density  : kg/m^3
    g            : gravitational acceleration (m/s^2)
    x0, y0, theta0 : initial pose

    Ports
    -----
    in:  velocity_left  (m/s), velocity_right (m/s)
    out: F_resist_left  (N) — feeds back to motor_L.F_resist
         F_resist_right (N) — feeds back to motor_R.F_resist
         velocity       (m/s, body-axis longitudinal, = (vL+vR)/2)
         yaw_rate       (rad/s)
         x, y           (m, world frame)
         theta          (rad, heading, wrapped to (-pi, pi])
    """

    def __init__(
        self,
        name: str,
        mass: float = 2.0,
        track_width: float = 0.20,
        Crr: float = 0.04,           # higher than racing car — rovers crawl
        Cd: float = 0.8,
        frontal_area: float = 0.020,
        scrub_coeff: float = 0.05,
        air_density: float = 1.225,
        g: float = 9.81,
        x0: float = 0.0,
        y0: float = 0.0,
        theta0: float = 0.0,
    ):
        super().__init__(name)
        if mass <= 0:
            raise ValueError("mass must be > 0")
        if track_width <= 0:
            raise ValueError("track_width must be > 0")
        if scrub_coeff < 0:
            raise ValueError("scrub_coeff must be >= 0")
        self.mass = mass
        self.track_width = track_width
        self.Crr = Crr
        self.Cd = Cd
        self.frontal_area = frontal_area
        self.scrub_coeff = scrub_coeff
        self.air_density = air_density
        self.g = g
        self.x0 = x0
        self.y0 = y0
        self.theta0 = theta0

        self.add_input("velocity_left")
        self.add_input("velocity_right")
        self.add_output("F_resist_left")
        self.add_output("F_resist_right")
        self.add_output("velocity")
        self.add_output("yaw_rate")
        self.add_output("x")
        self.add_output("y")
        self.add_output("theta")

        self.x = x0
        self.y = y0
        self.theta = theta0

    def reset(self) -> None:
        self.x = self.x0
        self.y = self.y0
        self.theta = self.theta0

    def step(self, t: float, dt: float) -> None:
        v_L = self.inputs["velocity_left"].value
        v_R = self.inputs["velocity_right"].value

        v = 0.5 * (v_L + v_R)
        yaw_rate = (v_R - v_L) / self.track_width

        # Pose integration (midpoint heading for fixed-dt accuracy).
        theta_mid = self.theta + 0.5 * yaw_rate * dt
        self.x += v * math.cos(theta_mid) * dt
        self.y += v * math.sin(theta_mid) * dt
        self.theta += yaw_rate * dt
        if self.theta > math.pi:
            self.theta -= 2.0 * math.pi
        elif self.theta <= -math.pi:
            self.theta += 2.0 * math.pi

        # Longitudinal resistance (rolling + aero), split per wheel.
        v_eps = 1e-3
        yaw_eps = 1e-2
        F_roll = self.Crr * self.mass * self.g * math.tanh(v / v_eps)
        F_drag = 0.5 * self.air_density * self.Cd * self.frontal_area * v * abs(v)
        F_long_half = 0.5 * (F_roll + F_drag)

        # Scrub: each wheel feels extra friction proportional to |yaw_rate|,
        # opposing its own direction of travel.
        scrub = self.scrub_coeff * self.mass * self.g * math.tanh(abs(yaw_rate) / yaw_eps)
        F_resist_left  = F_long_half + scrub * math.tanh(v_L / v_eps)
        F_resist_right = F_long_half + scrub * math.tanh(v_R / v_eps)

        self.outputs["F_resist_left"].value  = F_resist_left
        self.outputs["F_resist_right"].value = F_resist_right
        self.outputs["velocity"].value = v
        self.outputs["yaw_rate"].value = yaw_rate
        self.outputs["x"].value = self.x
        self.outputs["y"].value = self.y
        self.outputs["theta"].value = self.theta
