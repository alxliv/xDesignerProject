"""Encoder: quadrature optical/Hall encoder model.

Pulses-per-revolution (PPR) refers to one channel; with quadrature
decoding (X4) you get 4 * PPR counts per revolution. The encoder is
mounted on the *motor* shaft (pre-gearbox), as on the Yahboom 520.
"""
import math
from ..block import Block


class Encoder(Block):
    """Quadrature encoder model.

    Parameters
    ----------
    ppr        : pulses per revolution per channel (11 for Yahboom 520 Hall)
    quadrature : if True, count rate is 4 * ppr per motor revolution
    dt_sample  : optional sampling period (seconds) for the velocity
                 estimate. If None, velocity is reported every step.

    Ports
    -----
    in:  omega           (rad/s, motor shaft)
    out: count           (integer accumulated quadrature count)
         omega_measured  (rad/s, estimated from delta-count over sample dt)
    """

    def __init__(
        self,
        name: str,
        ppr: int = 11,
        quadrature: bool = True,
        dt_sample: float | None = None,
    ):
        super().__init__(name)
        self.ppr = ppr
        self.quadrature = quadrature
        self.counts_per_rev = (4 * ppr) if quadrature else ppr
        self.dt_sample = dt_sample

        self.add_input("omega")
        self.add_output("count")
        self.add_output("omega_measured")

        # state
        self._angle = 0.0
        self._last_count = 0
        self._last_sample_time = 0.0

    def reset(self) -> None:
        self._angle = 0.0
        self._last_count = 0
        self._last_sample_time = 0.0

    def step(self, t: float, dt: float) -> None:
        omega = self.inputs["omega"].value
        self._angle += omega * dt

        count = int(round(self._angle * self.counts_per_rev / (2.0 * math.pi)))
        self.outputs["count"].value = float(count)

        do_sample = self.dt_sample is None or (t - self._last_sample_time) >= self.dt_sample
        if do_sample:
            sample_dt = max(t - self._last_sample_time, dt)
            d_count = count - self._last_count
            measured = (d_count / self.counts_per_rev) * 2.0 * math.pi / sample_dt
            self.outputs["omega_measured"].value = measured
            self._last_count = count
            self._last_sample_time = t
