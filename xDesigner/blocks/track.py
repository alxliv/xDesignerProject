"""Track: closed (or open) centerline defined by 2D waypoints.

Reads the car position (x, y) and emits a track-frame view of where the
car is along the path: progress along the centerline, signed lateral
error, the tangent heading at the nearest point, and the local curvature.

Construction:

    track = Track("track", waypoints=[(x0, y0), (x1, y1), ...], closed=True)

or load from a TOML file (see ``Track.from_toml``).

In addition to the block ports, the ``Track`` instance exposes helper
methods (``lookahead_point``, ``length``) that other blocks — e.g. the
``PathFollower`` — can call imperatively, since they need access to
geometry that doesn't reduce to a scalar signal.
"""
from __future__ import annotations

import math
import tomllib
from pathlib import Path
from typing import List, Sequence, Tuple

from ..block import Block


Waypoint = Tuple[float, float]


def _norm_angle(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


class Track(Block):
    """Centerline-projecting track block.

    Parameters
    ----------
    name : block name
    waypoints : list of (x, y) tuples in metres. Must contain at least 2
                points. For closed tracks, do NOT repeat the start point at
                the end — the wrap is handled internally.
    closed : if True, the last segment connects back to the first point.
    search_window : when projecting (x, y) onto the centerline, search this
                    many segments either side of the previous best segment.
                    Set to None to search every segment each step (slower
                    but robust against teleportation). Defaults to None on
                    the first call and a small window thereafter.

    Ports
    -----
    in:  x (m), y (m)
    out: s_progress     — arc length along centerline from start (m)
         lateral_error  — signed perpendicular distance, left of path positive (m)
         heading_ref    — tangent angle of the path at the nearest point (rad)
         curvature      — local signed curvature 1/R at the nearest point (1/m)
    """

    def __init__(
        self,
        name: str,
        waypoints: Sequence[Waypoint],
        closed: bool = True,
        search_window: int | None = 5,
    ):
        super().__init__(name)
        if len(waypoints) < 2:
            raise ValueError("Track needs at least 2 waypoints")
        self.waypoints: List[Waypoint] = [(float(x), float(y)) for x, y in waypoints]
        self.closed = closed
        self.search_window = search_window

        self.add_input("x")
        self.add_input("y")
        self.add_output("s_progress")
        self.add_output("lateral_error")
        self.add_output("heading_ref")
        self.add_output("curvature")

        # Precompute segment geometry.
        self._precompute()

        # Search-state hint: index of last-best segment.
        self._last_seg: int | None = None

    # ---- construction helpers ---------------------------------------------
    @classmethod
    def from_toml(cls, path: str | Path, name: str | None = None) -> "Track":
        """Load a track definition from a TOML file.

        Expected schema::

            name = "figure-8"          # optional; used if `name` arg is None
            closed = true              # optional; default True
            waypoints = [
                [0.0, 0.0],
                [1.0, 0.0],
                ...
            ]
        """
        path = Path(path)
        with open(path, "rb") as f:
            data = tomllib.load(f)
        block_name = name or data.get("name") or path.stem
        closed = bool(data.get("closed", True))
        raw = data.get("waypoints")
        if raw is None:
            raise ValueError(f"{path}: missing 'waypoints' table")
        waypoints = [(float(pt[0]), float(pt[1])) for pt in raw]
        return cls(block_name, waypoints=waypoints, closed=closed)

    # ---- geometry precompute ----------------------------------------------
    def _precompute(self) -> None:
        pts = self.waypoints
        n = len(pts)
        n_seg = n if self.closed else n - 1
        self._n_seg = n_seg

        seg_dx: List[float] = []
        seg_dy: List[float] = []
        seg_len: List[float] = []
        seg_tan: List[float] = []  # tangent angle of each segment
        cum_s: List[float] = [0.0]

        for i in range(n_seg):
            x0, y0 = pts[i]
            x1, y1 = pts[(i + 1) % n]
            dx = x1 - x0
            dy = y1 - y0
            L = math.hypot(dx, dy)
            if L <= 0:
                raise ValueError(f"zero-length segment at index {i}")
            seg_dx.append(dx)
            seg_dy.append(dy)
            seg_len.append(L)
            seg_tan.append(math.atan2(dy, dx))
            cum_s.append(cum_s[-1] + L)

        self._seg_dx = seg_dx
        self._seg_dy = seg_dy
        self._seg_len = seg_len
        self._seg_tan = seg_tan
        self._cum_s = cum_s
        self._total_length = cum_s[-1]

        # Per-segment signed curvature: change in tangent / arc length,
        # averaged across the two segments meeting at the starting vertex.
        # For an open track, endpoints copy their neighbour.
        seg_curv: List[float] = [0.0] * n_seg
        for i in range(n_seg):
            i_prev = (i - 1) % n_seg if self.closed else max(i - 1, 0)
            dtheta = _norm_angle(seg_tan[i] - seg_tan[i_prev])
            ds = 0.5 * (seg_len[i] + seg_len[i_prev])
            seg_curv[i] = dtheta / ds if ds > 0 else 0.0
        self._seg_curv = seg_curv

    # ---- public geometry API ----------------------------------------------
    @property
    def length(self) -> float:
        """Total arc length of the (closed or open) centerline (m)."""
        return self._total_length

    def project(self, x: float, y: float, hint: int | None = None
                ) -> Tuple[int, float, float, float, float]:
        """Project (x, y) onto the centerline.

        Returns (best_seg, t_along, dist_signed, s_progress, tangent_angle).
        ``t_along`` is in [0, 1] within the chosen segment. ``dist_signed``
        is positive when the point is to the LEFT of the path direction.
        """
        n_seg = self._n_seg
        if self.search_window is None or hint is None:
            indices = range(n_seg)
        else:
            w = self.search_window
            if self.closed:
                indices = [(hint + k) % n_seg for k in range(-w, w + 1)]
            else:
                lo = max(0, hint - w)
                hi = min(n_seg, hint + w + 1)
                indices = range(lo, hi)

        best = None  # (dist_sq, seg, t)
        for i in indices:
            x0, y0 = self.waypoints[i]
            dx = self._seg_dx[i]
            dy = self._seg_dy[i]
            L = self._seg_len[i]
            # parametric projection, clamped to segment
            t = ((x - x0) * dx + (y - y0) * dy) / (L * L)
            t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
            px = x0 + t * dx
            py = y0 + t * dy
            d2 = (x - px) ** 2 + (y - py) ** 2
            if best is None or d2 < best[0]:
                best = (d2, i, t)

        assert best is not None
        _, seg, t = best
        x0, y0 = self.waypoints[seg]
        dx = self._seg_dx[seg]
        dy = self._seg_dy[seg]
        L = self._seg_len[seg]
        px = x0 + t * dx
        py = y0 + t * dy
        # signed lateral: left of path direction is positive (2D cross product)
        # path direction = (dx, dy)/L; offset = (x - px, y - py)
        signed = ((x - px) * (-dy) + (y - py) * dx) / L
        s_progress = self._cum_s[seg] + t * L
        return seg, t, signed, s_progress, self._seg_tan[seg]

    def lookahead_point(self, x: float, y: float, lookahead: float,
                        hint: int | None = None) -> Tuple[float, float]:
        """Return a point on the centerline ~`lookahead` metres ahead
        of the current projection of (x, y).

        Walks forward along the centerline until the cumulative arc length
        from the projection exceeds `lookahead`, then interpolates within
        the final segment.
        """
        n_seg = self._n_seg
        seg, t, _, _, _ = self.project(x, y, hint=hint)
        x0, y0 = self.waypoints[seg]
        L0 = self._seg_len[seg]
        # remaining length in current segment from projection forward
        remain = (1.0 - t) * L0
        if remain >= lookahead:
            t_target = t + lookahead / L0
            return (x0 + t_target * self._seg_dx[seg],
                    y0 + t_target * self._seg_dy[seg])
        accumulated = remain
        i = seg
        while True:
            i_next = (i + 1) % n_seg if self.closed else i + 1
            if not self.closed and i_next >= n_seg:
                # ran off the end of an open track — clamp to last point
                xe, ye = self.waypoints[-1]
                return xe, ye
            i = i_next
            Li = self._seg_len[i]
            if accumulated + Li >= lookahead:
                need = lookahead - accumulated
                tt = need / Li
                xi, yi = self.waypoints[i]
                return (xi + tt * self._seg_dx[i],
                        yi + tt * self._seg_dy[i])
            accumulated += Li

    # ---- block lifecycle --------------------------------------------------
    def reset(self) -> None:
        self._last_seg = None

    def step(self, t_sim: float, dt: float) -> None:
        x = self.inputs["x"].value
        y = self.inputs["y"].value
        seg, _, signed, s_progress, tangent = self.project(x, y, hint=self._last_seg)
        self._last_seg = seg
        self.outputs["s_progress"].value = s_progress
        self.outputs["lateral_error"].value = signed
        self.outputs["heading_ref"].value = tangent
        self.outputs["curvature"].value = self._seg_curv[seg]
