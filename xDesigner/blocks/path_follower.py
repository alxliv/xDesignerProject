"""PathFollower: geometric pure-pursuit steering controller.

Reads the car pose (x, y, theta) and longitudinal speed, queries a
``Track`` instance for a lookahead point a few car-lengths ahead of the
current projection on the centerline, and computes the steering angle
that would carry the rear axle through that point along a circular arc::

    L_d   = max(lookahead_min, lookahead_k · v)
    α     = angle to lookahead point, relative to vehicle heading
    δ     = atan2(2 · L · sin(α), L_d)

The output is normalised to [-1, +1] by ``delta_max`` so it can plug
straight into a ``Steering`` block.
"""
from __future__ import annotations

import math

from ..block import Block
from .track import Track, _norm_angle


class PathFollower(Block):
    """Pure-pursuit path-tracking controller.

    Parameters
    ----------
    track          : Track instance whose centerline will be followed
    wheelbase      : front-to-rear axle distance L (m), must match Chassis2D
    delta_max      : maximum steering angle (rad) used to normalise the output
    lookahead_k    : lookahead distance per unit of speed (s)
    lookahead_min  : minimum lookahead distance (m) — prevents shrinking to 0
                     at standstill
    lookahead_max  : optional clamp on the upper end of lookahead distance (m)

    Ports
    -----
    in:  x (m), y (m), theta (rad), v (m/s)
    out: steering_command (dimensionless, in [-1, +1])
    """

    def __init__(
        self,
        name: str,
        track: Track,
        wheelbase: float,
        delta_max: float = math.radians(30.0),
        lookahead_k: float = 0.4,
        lookahead_min: float = 0.25,
        lookahead_max: float | None = None,
    ):
        super().__init__(name)
        if wheelbase <= 0:
            raise ValueError("wheelbase must be > 0")
        if delta_max <= 0:
            raise ValueError("delta_max must be > 0")
        if lookahead_min <= 0:
            raise ValueError("lookahead_min must be > 0")
        self.track = track
        self.wheelbase = wheelbase
        self.delta_max = delta_max
        self.lookahead_k = lookahead_k
        self.lookahead_min = lookahead_min
        self.lookahead_max = lookahead_max

        self.add_input("x")
        self.add_input("y")
        self.add_input("theta")
        self.add_input("v")
        self.add_output("steering_command")

        self._seg_hint: int | None = None

    def reset(self) -> None:
        self._seg_hint = None

    def step(self, t: float, dt: float) -> None:
        x = self.inputs["x"].value
        y = self.inputs["y"].value
        theta = self.inputs["theta"].value
        v = self.inputs["v"].value

        L_d = self.lookahead_min + self.lookahead_k * max(v, 0.0)
        if self.lookahead_max is not None:
            L_d = min(L_d, self.lookahead_max)

        # update the segment hint via a cheap projection so lookahead
        # walks forward from the right place when the path crosses itself
        seg, _, _, _, _ = self.track.project(x, y, hint=self._seg_hint)
        self._seg_hint = seg

        tx, ty = self.track.lookahead_point(x, y, L_d, hint=seg)
        dx = tx - x
        dy = ty - y
        # angle to lookahead point, relative to the car heading
        alpha = _norm_angle(math.atan2(dy, dx) - theta)

        delta = math.atan2(2.0 * self.wheelbase * math.sin(alpha), L_d)

        # normalise to [-1, +1] then clip
        cmd = delta / self.delta_max
        if cmd > 1.0:
            cmd = 1.0
        elif cmd < -1.0:
            cmd = -1.0
        self.outputs["steering_command"].value = cmd
