"""Track tests: TOML loading, projection, lookahead, crossing-robustness."""
import math
import os

import pytest

from xDesigner.blocks import Track


HERE = os.path.dirname(os.path.abspath(__file__))
FIG8 = os.path.normpath(os.path.join(HERE, "..", "examples", "tracks",
                                     "figure8.toml"))


def test_validates_short_waypoint_list():
    with pytest.raises(ValueError):
        Track("t", waypoints=[(0.0, 0.0)])


def test_rejects_zero_length_segment():
    with pytest.raises(ValueError):
        Track("t", waypoints=[(0.0, 0.0), (0.0, 0.0), (1.0, 0.0)],
              closed=False)


def test_unit_square_geometry():
    """A 1×1 closed square: perimeter 4, tangent of bottom edge is 0."""
    t = Track("sq",
              waypoints=[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)],
              closed=True)
    assert t.length == pytest.approx(4.0)
    # Projection of (0.5, -0.5) lands on segment 0 at t=0.5; the point is
    # to the RIGHT of the +x tangent → signed lateral negative.
    seg, tt, signed, s, tan = t.project(0.5, -0.5)
    assert seg == 0
    assert tt == pytest.approx(0.5)
    assert signed == pytest.approx(-0.5)
    assert s == pytest.approx(0.5)
    assert tan == pytest.approx(0.0)


def test_lateral_error_sign_convention_is_left_positive():
    """Horizontal segment (0,0)→(1,0): tangent is +x, so a point ABOVE
    (positive y) sits to the LEFT of the direction of travel → positive
    signed lateral error."""
    t = Track("seg", waypoints=[(0.0, 0.0), (1.0, 0.0)], closed=False)
    seg, _, signed, _, _ = t.project(0.5, 0.1)
    assert seg == 0
    assert signed == pytest.approx(0.1)   # +y is left, left is positive


def test_lookahead_walks_forward_through_segments():
    """Square track: from (0.1, 0) with lookahead 1.5 m, we should land
    on segment 1 (the right edge), 0.6 m up from (1, 0)."""
    t = Track("sq",
              waypoints=[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)],
              closed=True)
    px, py = t.lookahead_point(0.1, 0.0, 1.5)
    # 0.9 m to reach (1,0), then 0.6 m up: (1.0, 0.6)
    assert px == pytest.approx(1.0)
    assert py == pytest.approx(0.6)


def test_lookahead_wraps_on_closed_track():
    """Lookahead longer than the path remaining must wrap past s=0."""
    t = Track("sq",
              waypoints=[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)],
              closed=True)
    # From (0.5, 1) on segment 2, walk 3.0 m → lands on segment 1 (right edge).
    # remaining seg2: 0.5; seg3: 1.0; seg0: 1.0; need 0.5 more on seg1 → (1.0, 0.5)
    px, py = t.lookahead_point(0.5, 1.0, 3.0)
    assert px == pytest.approx(1.0)
    assert py == pytest.approx(0.5)


def test_figure8_toml_loads_and_has_expected_perimeter():
    t = Track.from_toml(FIG8)
    assert len(t.waypoints) == 60
    # Bernoulli lemniscate with a=1.2 has perimeter ≈ 5.244·a = 6.293 m;
    # piecewise-linear with 60 samples loses a hair (~6.283 m).
    assert t.length == pytest.approx(6.28, abs=0.05)


def test_figure8_projection_hint_distinguishes_crossing():
    """At the figure-8 crossing (the origin) two non-adjacent segments
    intersect. With a segment hint, ``project`` must prefer the segment
    near the hint over the geometrically-equally-near one on the other
    loop."""
    t = Track.from_toml(FIG8)
    # Find the two segments closest to the origin by raw search.
    # Then verify that projecting (0,0) with each as a hint returns
    # different segment indices.
    seg_a, *_ = t.project(0.0, 0.0, hint=None)
    # Choose a hint segment several waypoints away on the OTHER loop.
    # The lemniscate sample crosses origin near indices ~15 and ~45 with
    # 60 points; pick hints well inside each branch.
    seg_b, *_ = t.project(0.0, 0.0, hint=(seg_a + 30) % t._n_seg)
    assert seg_a != seg_b, "Hint should pull projection toward the hinted loop"


def test_reset_clears_segment_hint():
    t = Track.from_toml(FIG8)
    t.inputs["x"].value = 1.2
    t.inputs["y"].value = 0.0
    t.step(0.0, 1e-3)
    assert t._last_seg is not None
    t.reset()
    assert t._last_seg is None


def test_open_track_lookahead_clamps_at_endpoint():
    t = Track("line", waypoints=[(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)],
              closed=False)
    px, py = t.lookahead_point(0.0, 0.0, 100.0)
    assert px == pytest.approx(2.0)
    assert py == pytest.approx(0.0)
