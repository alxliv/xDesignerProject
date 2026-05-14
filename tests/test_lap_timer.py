"""LapTimer tests: wrap detection, min_lap_time guard, best-lap tracking."""
import pytest

from xDesigner.blocks import LapTimer


def _feed(lap, s_sequence, dt):
    """Drive the lap timer through a list of (t, s_progress) pairs."""
    outputs = []
    for i, s in enumerate(s_sequence):
        lap.inputs["s_progress"].value = s
        lap.step(t=i * dt, dt=dt)
        outputs.append({
            "lap_count": lap.outputs["lap_count"].value,
            "last":     lap.outputs["last_lap_time"].value,
            "best":     lap.outputs["best_lap_time"].value,
        })
    return outputs


def test_validates_track_length():
    with pytest.raises(ValueError):
        LapTimer("l", track_length=0.0)


def test_no_laps_before_first_wrap():
    lap = LapTimer("l", track_length=10.0)
    out = _feed(lap, [0.0, 1.0, 2.0, 3.0], dt=0.1)
    assert all(o["lap_count"] == 0 for o in out)


def test_single_wrap_counts_one_lap():
    lap = LapTimer("l", track_length=10.0, min_lap_time=0.0)
    # s walks 0 → 9 then wraps to 0; the moment it goes 9 → 0, lap += 1.
    s = list(range(10)) + [0, 1, 2]  # 0..9, then wrap
    out = _feed(lap, [float(x) for x in s], dt=0.1)
    assert out[-1]["lap_count"] == 1
    # Wrap occurred at index 10; that's t = 10*0.1 = 1.0 s, but LapTimer
    # starts its clock at step 0, so elapsed at wrap = 1.0 s.
    assert out[-1]["last"] == pytest.approx(1.0, abs=0.05)


def test_min_lap_time_guard_rejects_spurious_wraps():
    """A bouncy s_progress that jumps backward briefly must NOT count
    if min_lap_time hasn't elapsed."""
    lap = LapTimer("l", track_length=10.0, min_lap_time=2.0)
    # Walk forward to s=9, immediately bounce to s=0 (a spurious wrap):
    # since elapsed time is well under 2.0 s, the wrap should be ignored.
    s = [0.0, 5.0, 9.0, 0.0, 5.0]
    out = _feed(lap, s, dt=0.1)
    assert all(o["lap_count"] == 0 for o in out)


def test_best_lap_tracks_minimum_across_multiple_laps():
    lap = LapTimer("l", track_length=10.0, min_lap_time=0.0)
    dt = 0.01
    # Lap 1: 100 steps from 0 to 9.9, then wrap. Elapsed = 100 * 0.01 = 1.0 s.
    # Lap 2: 200 steps, slower. Elapsed = 2.0 s.
    # Lap 3: 50 steps, faster. Elapsed = 0.5 s.
    s = []
    s += [i * 0.1 for i in range(100)]    # 0.0 .. 9.9
    s += [i * 0.05 for i in range(200)]   # 0.0 .. 9.95
    s += [i * 0.2 for i in range(50)]     # 0.0 .. 9.8
    s += [0.0]                            # final wrap
    out = _feed(lap, s, dt=dt)
    assert out[-1]["lap_count"] == 3
    assert out[-1]["best"] == pytest.approx(0.5, abs=0.05)


def test_reset_clears_lap_state():
    lap = LapTimer("l", track_length=10.0, min_lap_time=0.0)
    _feed(lap, [0.0, 5.0, 9.0, 0.0, 5.0], dt=0.1)
    assert lap.outputs["lap_count"].value == 1
    lap.reset()
    assert lap.outputs["lap_count"].value == 1  # output not re-emitted yet
    # but internal state is reset:
    assert lap._lap_count == 0
    assert lap._initialised is False
