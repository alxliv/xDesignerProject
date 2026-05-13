"""MotorDriver: maps a normalized command in [-1, +1] to a motor voltage."""
from ..block import Block


class MotorDriver(Block):
    """H-bridge / PWM motor driver model.

    Parameters
    ----------
    vbus : DC bus voltage (volts).
    deadband : magnitude below which output is forced to 0 (models H-bridge
        deadtime / switching deadband).
    voltage_drop : voltage drop across the bridge at full duty (volts).

    Ports
    -----
    in:  command  (expected in [-1, +1])
    out: voltage  (volts)
    """

    def __init__(
        self,
        name: str,
        vbus: float = 12.0,
        deadband: float = 0.0,
        voltage_drop: float = 0.0,
    ):
        super().__init__(name)
        self.vbus = vbus
        self.deadband = deadband
        self.voltage_drop = voltage_drop

        self.add_input("command")
        self.add_output("voltage")

    def step(self, t: float, dt: float) -> None:
        u = self.inputs["command"].value
        u = max(-1.0, min(1.0, u))
        if abs(u) < self.deadband:
            v = 0.0
        else:
            v = u * (self.vbus - self.voltage_drop)
        self.outputs["voltage"].value = v
