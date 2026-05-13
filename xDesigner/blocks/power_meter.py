"""PowerMeter: instantaneous and cumulative electrical power on a motor.

Wire one of these in parallel with each motor (voltage from the driver,
current from the motor) and probe ``energy`` for the total joules
delivered to that motor over the run. The Mars-rover demo sums two of
them to score "least power wins".
"""
from ..block import Block


class PowerMeter(Block):
    """Instantaneous and integrated electrical power.

    Parameters
    ----------
    use_abs : if True (default), integrate |V·I| so regenerative braking
              (a motor that briefly returns current) still counts as
              energy spent at the battery. Set False to allow negative
              contributions to the integral.

    Ports
    -----
    in:  voltage (V), current (A)
    out: power (W, signed = V·I or |V·I| depending on use_abs)
         energy (J, integrated power over time since reset)
    """

    def __init__(self, name: str, use_abs: bool = True):
        super().__init__(name)
        self.use_abs = use_abs

        self.add_input("voltage")
        self.add_input("current")
        self.add_output("power")
        self.add_output("energy")

        self._energy = 0.0

    def reset(self) -> None:
        self._energy = 0.0

    def step(self, t: float, dt: float) -> None:
        V = self.inputs["voltage"].value
        I = self.inputs["current"].value
        P = V * I
        if self.use_abs:
            P = abs(P)
        self._energy += P * dt
        self.outputs["power"].value = P
        self.outputs["energy"].value = self._energy
