"""SignalSource: generates a reference waveform (step / ramp / sine / const)."""
import math
from ..block import Block


class SignalSource(Block):
    """Reference signal generator.

    Parameters
    ----------
    waveform : 'step' | 'ramp' | 'sine' | 'const'
    amplitude : signal amplitude (for step/sine/const)
    offset    : DC offset added to the signal
    t_start   : time at which the waveform begins (before this, output = offset)
    frequency : sine frequency in Hz
    slope     : ramp slope (units/s)
    """

    def __init__(
        self,
        name: str,
        waveform: str = "step",
        amplitude: float = 1.0,
        offset: float = 0.0,
        t_start: float = 0.0,
        frequency: float = 1.0,
        slope: float = 1.0,
    ):
        super().__init__(name)
        if waveform not in ("step", "ramp", "sine", "const"):
            raise ValueError(f"unknown waveform {waveform!r}")
        self.waveform = waveform
        self.amplitude = amplitude
        self.offset = offset
        self.t_start = t_start
        self.frequency = frequency
        self.slope = slope
        self.add_output("value", default=offset)

    def step(self, t: float, dt: float) -> None:
        if t < self.t_start:
            v = self.offset
        elif self.waveform == "step":
            v = self.offset + self.amplitude
        elif self.waveform == "ramp":
            v = self.offset + self.slope * (t - self.t_start)
        elif self.waveform == "sine":
            v = self.offset + self.amplitude * math.sin(
                2.0 * math.pi * self.frequency * (t - self.t_start)
            )
        else:  # const
            v = self.offset + self.amplitude
        self.outputs["value"].value = v
