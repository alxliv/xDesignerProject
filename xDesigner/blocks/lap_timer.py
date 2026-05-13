"""LapTimer: detects laps by watching arc-length progress on a closed track.

The ``Track`` block emits ``s_progress`` in [0, track_length). On a closed
track, a lap completes when ``s_progress`` wraps from near ``track_length``
back to near 0. We detect that wrap (a large negative jump), record the
elapsed time since the last wrap, and keep a running best.

To avoid spurious double-counts from numerical noise near the start/finish
line, the timer enforces a minimum lap duration ``min_lap_time``.
"""
from ..block import Block


class LapTimer(Block):
    """Lap counter and per-lap timer driven by ``s_progress``.

    Parameters
    ----------
    track_length  : total arc length of the closed centerline (m).
    min_lap_time  : minimum elapsed time before a wrap is counted as a
                    new lap (s). Defaults to 0.5 s.
    wrap_fraction : a wrap is recognised when s drops by more than
                    ``wrap_fraction · track_length`` between two steps.
                    Defaults to 0.5 (half a lap), which is robust as long
                    as the car can't sim more than half a lap per step.

    Ports
    -----
    in:  s_progress (m)
    out: lap_count        (int as float)
         current_lap_time (s, time elapsed since last start/finish crossing)
         last_lap_time    (s, duration of the most recently completed lap;
                           0.0 until the first lap completes)
         best_lap_time    (s, fastest completed lap; 0.0 until the first
                           lap completes)
    """

    def __init__(
        self,
        name: str,
        track_length: float,
        min_lap_time: float = 0.5,
        wrap_fraction: float = 0.5,
    ):
        super().__init__(name)
        if track_length <= 0:
            raise ValueError("track_length must be > 0")
        self.track_length = track_length
        self.min_lap_time = min_lap_time
        self.wrap_threshold = wrap_fraction * track_length

        self.add_input("s_progress")
        self.add_output("lap_count")
        self.add_output("current_lap_time")
        self.add_output("last_lap_time")
        self.add_output("best_lap_time")

        self._prev_s = 0.0
        self._lap_start_t = 0.0
        self._lap_count = 0
        self._last_lap = 0.0
        self._best_lap = 0.0
        self._initialised = False

    def reset(self) -> None:
        self._prev_s = 0.0
        self._lap_start_t = 0.0
        self._lap_count = 0
        self._last_lap = 0.0
        self._best_lap = 0.0
        self._initialised = False

    def step(self, t: float, dt: float) -> None:
        s = self.inputs["s_progress"].value
        if not self._initialised:
            self._prev_s = s
            self._lap_start_t = t
            self._initialised = True

        # Wrap detection: s_progress dropped sharply → crossed start/finish.
        elapsed = t - self._lap_start_t
        if (self._prev_s - s) > self.wrap_threshold and elapsed >= self.min_lap_time:
            self._lap_count += 1
            self._last_lap = elapsed
            if self._best_lap == 0.0 or elapsed < self._best_lap:
                self._best_lap = elapsed
            self._lap_start_t = t

        self._prev_s = s

        self.outputs["lap_count"].value = float(self._lap_count)
        self.outputs["current_lap_time"].value = t - self._lap_start_t
        self.outputs["last_lap_time"].value = self._last_lap
        self.outputs["best_lap_time"].value = self._best_lap
