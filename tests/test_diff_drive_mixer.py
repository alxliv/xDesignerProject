"""DiffDriveMixer tests: mixing math + saturation behaviour."""
import pytest

from xDesigner.blocks import DiffDriveMixer


def _mix(mixer, throttle, steering):
    mixer.inputs["throttle"].value = throttle
    mixer.inputs["steering"].value = steering
    mixer.step(0.0, 1e-3)
    return (mixer.outputs["command_left"].value,
            mixer.outputs["command_right"].value)


def test_validates_mixing_gain():
    with pytest.raises(ValueError):
        DiffDriveMixer("m", mixing_gain=0.0)
    with pytest.raises(ValueError):
        DiffDriveMixer("m", mixing_gain=-0.5)


def test_zero_steering_passes_throttle_to_both_wheels():
    m = DiffDriveMixer("m", mixing_gain=0.7)
    cL, cR = _mix(m, throttle=0.5, steering=0.0)
    assert cL == pytest.approx(0.5)
    assert cR == pytest.approx(0.5)


def test_positive_steering_speeds_right_slows_left():
    """Convention: +steering should turn left (yaw counter-clockwise),
    which requires right wheel faster than left."""
    m = DiffDriveMixer("m", mixing_gain=0.5)
    cL, cR = _mix(m, throttle=0.0, steering=0.6)
    assert cR > cL
    assert cR == pytest.approx(0.30)
    assert cL == pytest.approx(-0.30)


def test_full_throttle_full_steering_saturates_one_wheel():
    """At throttle=+1, steering=+1, k=1: right wants +2 (clips to +1),
    left wants 0. The CAP on the right side means we trade forward
    speed for yaw — exactly the intuitive behaviour."""
    m = DiffDriveMixer("m", mixing_gain=1.0)
    cL, cR = _mix(m, throttle=1.0, steering=1.0)
    assert cR == pytest.approx(1.0)
    assert cL == pytest.approx(0.0)


def test_inputs_above_unit_are_clamped_before_mixing():
    """Garbage in upstream shouldn't blow the mixer up."""
    m = DiffDriveMixer("m", mixing_gain=0.5)
    cL, cR = _mix(m, throttle=10.0, steering=-10.0)
    # throttle clamps to 1, steering to -1; cmd_L = 1 - 0.5·(-1) = 1.5 → clip → 1.
    # cmd_R = 1 + 0.5·(-1) = 0.5
    assert cL == pytest.approx(1.0)
    assert cR == pytest.approx(0.5)


def test_symmetry_negating_steering_swaps_wheels():
    m = DiffDriveMixer("m", mixing_gain=0.7)
    cL1, cR1 = _mix(m, throttle=0.3, steering=0.4)
    cL2, cR2 = _mix(m, throttle=0.3, steering=-0.4)
    assert cL1 == pytest.approx(cR2)
    assert cR1 == pytest.approx(cL2)
