"""DCMotor: brushed DC motor with embedded gearbox and reflected load mass.

Models a Yahboom 520-class motor by default. The "load" is represented as
a longitudinal vehicle: the motor's mechanical state is its angular
velocity, but the effective inertia includes the reflected mass of the
chassis seen through the gearbox + wheel.

Electrical:  V = R*i + L*di/dt + ke*omega_motor
Mechanical:  J_eff * d(omega_motor)/dt = kt*i - b*omega_motor - tau_resist

with
    J_eff       = J_rotor + reflected_mass * (wheel_radius / gear_ratio)**2
    tau_resist  = F_resist * wheel_radius / (gear_ratio * efficiency)
    omega_out   = omega_motor / gear_ratio
    velocity    = omega_motor * wheel_radius / gear_ratio

The electrical equation is integrated *analytically* over each step
(treating V and omega as constant during dt) so the step size is not
limited by the small L/R electrical time constant.
"""
import math
from ..block import Block


class DCMotor(Block):
    """Brushed DC motor + gearbox + reflected vehicle mass.

    Default parameters approximate a Yahboom 520 motor with 1:30 reduction
    (333 RPM no-load at 12 V) driving a 65 mm-diameter wheel and a 1 kg
    vehicle.

    Parameters
    ----------
    R, L      : armature resistance (ohm) and inductance (H)
    ke, kt    : back-EMF constant (V*s/rad) and torque constant (N*m/A)
                (in SI they are numerically equal for an ideal motor)
    J_rotor   : rotor inertia (kg*m^2), as seen on the *motor* shaft (pre-gearbox)
    b         : viscous friction on motor shaft (N*m*s/rad)
    gear_ratio   : reduction ratio (motor turns per output turn), N >= 1
    efficiency   : gearbox + drivetrain mechanical efficiency in [0, 1]
    wheel_radius : effective rolling radius of the driven wheel (m)
    reflected_mass : vehicle mass reflected through this drivetrain (kg).
                     For a 1-motor car, this is the vehicle mass; for an
                     n-motor car, set this to (mass / n_motors).

    Ports
    -----
    in:
        voltage          : armature terminal voltage (V)
        F_resist         : resistive longitudinal force on vehicle (N)
                           (rolling resistance + aero drag + brake force ...)
    out:
        omega_motor      : motor-shaft angular velocity (rad/s), pre-gearbox
        omega_out        : output-shaft angular velocity (rad/s) = wheel rate
        current          : armature current (A)
        torque           : electromagnetic torque (N*m), at the motor shaft
        velocity         : vehicle longitudinal velocity (m/s)
    """

    def __init__(
        self,
        name: str,
        R: float = 2.0,
        L: float = 1.0e-3,
        ke: float = 0.0115,
        kt: float = 0.0115,
        J_rotor: float = 1.0e-5,
        b: float = 1.0e-6,
        gear_ratio: float = 30.0,
        efficiency: float = 0.85,
        wheel_radius: float = 0.0325,
        reflected_mass: float = 1.0,
    ):
        super().__init__(name)
        if R <= 0:
            raise ValueError("R must be > 0")
        if L <= 0:
            raise ValueError("L must be > 0")
        if gear_ratio <= 0:
            raise ValueError("gear_ratio must be > 0")
        if not (0 < efficiency <= 1):
            raise ValueError("efficiency must be in (0, 1]")

        self.R = R
        self.L = L
        self.ke = ke
        self.kt = kt
        self.J_rotor = J_rotor
        self.b = b
        self.gear_ratio = gear_ratio
        self.efficiency = efficiency
        self.wheel_radius = wheel_radius
        self.reflected_mass = reflected_mass

        self.add_input("voltage")
        self.add_input("F_resist")

        self.add_output("omega_motor")
        self.add_output("omega_out")
        self.add_output("current")
        self.add_output("torque")
        self.add_output("velocity")

        # state
        self.i = 0.0
        self.omega_motor = 0.0

    def reset(self) -> None:
        self.i = 0.0
        self.omega_motor = 0.0

    def step(self, t: float, dt: float) -> None:
        V = self.inputs["voltage"].value
        F_resist = self.inputs["F_resist"].value

        N = self.gear_ratio
        r = self.wheel_radius
        eta = self.efficiency

        # reflect resistive force back to motor shaft
        # Rolling/drag opposes velocity, so it appears as a positive load
        # when omega_motor is positive. F_resist already carries its sign.
        tau_resist = F_resist * r / (N * eta)

        # effective inertia at motor shaft
        J_eff = self.J_rotor + self.reflected_mass * (r / N) ** 2

        # ---- electrical (analytic over dt, V & omega held constant) ------
        # i' + (R/L) i = (V - ke*omega)/L  →  exponential approach to i_inf
        i_inf = (V - self.ke * self.omega_motor) / self.R
        alpha = math.exp(-dt * self.R / self.L)
        self.i = self.i * alpha + i_inf * (1.0 - alpha)

        # ---- mechanical (forward Euler is plenty for typical b/J) --------
        tau_motor = self.kt * self.i
        domega = (tau_motor - self.b * self.omega_motor - tau_resist) / J_eff
        self.omega_motor += domega * dt

        # outputs
        self.outputs["omega_motor"].value = self.omega_motor
        self.outputs["omega_out"].value = self.omega_motor / N
        self.outputs["current"].value = self.i
        self.outputs["torque"].value = tau_motor
        self.outputs["velocity"].value = self.omega_motor * r / N
