"""Kinematic sanity tests for both 2D chassis blocks.

The integrators here are forward-Euler (with midpoint heading), so we
allow modest tolerance — we're testing that the kinematics are right,
not that the integrator is perfect.
"""
import math

import pytest

from xDesigner.blocks import Chassis2D, DifferentialChassis


# --- Chassis2D (kinematic bicycle) -------------------------------------------

def _run_chassis2d(chassis, v, delta, duration, dt):
    chassis.inputs["velocity"].value = v
    chassis.inputs["delta"].value = delta
    n = int(round(duration / dt))
    for i in range(n):
        chassis.step(i * dt, dt)


def test_chassis2d_straight_line():
    """Zero steering at constant velocity: drive straight along +x."""
    c = Chassis2D("c", wheelbase=0.15)
    _run_chassis2d(c, v=1.0, delta=0.0, duration=2.0, dt=1e-3)
    assert c.x == pytest.approx(2.0, rel=1e-3)
    assert c.y == pytest.approx(0.0, abs=1e-6)
    assert c.theta == pytest.approx(0.0, abs=1e-6)


def test_chassis2d_drives_a_circle_of_expected_radius():
    """At steering angle δ, the kinematic bicycle traces a circle of
    radius R = wheelbase / tan(δ). After one full revolution the pose
    should return near the origin and heading."""
    L = 0.15
    delta = math.radians(20.0)
    R = L / math.tan(delta)
    v = 0.5
    period = 2 * math.pi * R / v   # time for one revolution
    c = Chassis2D("c", wheelbase=L)
    _run_chassis2d(c, v=v, delta=delta, duration=period, dt=5e-4)
    # back to (approximately) origin
    assert c.x == pytest.approx(0.0, abs=R * 0.02)
    assert c.y == pytest.approx(0.0, abs=R * 0.02)
    # heading wrapped to (-π, π] — should be near 0 or near ±π
    assert abs(c.theta) < 0.05 or abs(abs(c.theta) - math.pi) > math.pi - 0.05


def test_chassis2d_yaw_rate_matches_bicycle_formula():
    """yaw_rate = v · tan(δ) / L at the instant of the step."""
    L = 0.15
    delta = math.radians(15.0)
    v = 0.7
    c = Chassis2D("c", wheelbase=L)
    c.inputs["velocity"].value = v
    c.inputs["delta"].value = delta
    c.step(0.0, 1e-3)
    expected = v * math.tan(delta) / L
    assert c.outputs["yaw_rate"].value == pytest.approx(expected, rel=1e-6)


def test_chassis2d_reset_returns_to_initial_pose():
    c = Chassis2D("c", x0=1.5, y0=-0.5, theta0=math.pi / 4)
    _run_chassis2d(c, v=0.4, delta=0.1, duration=0.5, dt=1e-3)
    assert (c.x, c.y) != (1.5, -0.5)
    c.reset()
    assert c.x == 1.5 and c.y == -0.5 and c.theta == pytest.approx(math.pi / 4)


# --- DifferentialChassis (skid-steer) ----------------------------------------

def _run_diff(chassis, vL, vR, duration, dt):
    chassis.inputs["velocity_left"].value = vL
    chassis.inputs["velocity_right"].value = vR
    n = int(round(duration / dt))
    for i in range(n):
        chassis.step(i * dt, dt)


def test_diff_chassis_equal_wheels_go_straight():
    c = DifferentialChassis("c", track_width=0.2)
    _run_diff(c, vL=0.3, vR=0.3, duration=2.0, dt=1e-3)
    assert c.x == pytest.approx(0.6, rel=1e-3)
    assert c.y == pytest.approx(0.0, abs=1e-6)
    assert c.theta == pytest.approx(0.0, abs=1e-6)
    assert c.outputs["yaw_rate"].value == pytest.approx(0.0, abs=1e-6)


def test_diff_chassis_in_place_rotation():
    """Opposite-sign wheels at equal magnitude: zero longitudinal velocity,
    pure yaw at rate (vR - vL) / T."""
    T = 0.20
    c = DifferentialChassis("c", track_width=T, scrub_coeff=0.0)
    _run_diff(c, vL=-0.1, vR=0.1, duration=0.5, dt=1e-3)
    # No translation
    assert c.x == pytest.approx(0.0, abs=1e-3)
    assert c.y == pytest.approx(0.0, abs=1e-3)
    # Expected yaw rate: 0.2 / 0.2 = 1 rad/s; after 0.5 s, theta ≈ 0.5 rad.
    assert c.theta == pytest.approx(0.5, rel=2e-2)


def test_diff_chassis_yaw_rate_formula():
    T = 0.20
    c = DifferentialChassis("c", track_width=T)
    c.inputs["velocity_left"].value = 0.10
    c.inputs["velocity_right"].value = 0.30
    c.step(0.0, 1e-3)
    # yaw = (vR - vL) / T = 0.20 / 0.20 = 1.0 rad/s
    assert c.outputs["yaw_rate"].value == pytest.approx(1.0, rel=1e-6)
    # v = average of wheels = 0.20 m/s
    assert c.outputs["velocity"].value == pytest.approx(0.20, rel=1e-6)


def test_diff_chassis_scrub_costs_resistance_when_turning():
    """At zero translation but yawing, F_resist on each wheel should
    pick up the scrub term (in the direction of each wheel's motion)."""
    c_no_scrub = DifferentialChassis("c0", track_width=0.2, scrub_coeff=0.0)
    c_scrub    = DifferentialChassis("c1", track_width=0.2, scrub_coeff=0.05)
    for c in (c_no_scrub, c_scrub):
        c.inputs["velocity_left"].value = -0.1
        c.inputs["velocity_right"].value = 0.1
        c.step(0.0, 1e-3)
    # With scrub on, right wheel (positive v) sees higher resistance;
    # left wheel (negative v) sees more negative resistance — magnitudes
    # exceed the no-scrub case.
    assert abs(c_scrub.outputs["F_resist_right"].value) > \
           abs(c_no_scrub.outputs["F_resist_right"].value)
    assert abs(c_scrub.outputs["F_resist_left"].value) > \
           abs(c_no_scrub.outputs["F_resist_left"].value)


def test_diff_chassis_validates_inputs():
    with pytest.raises(ValueError):
        DifferentialChassis("c", mass=0.0)
    with pytest.raises(ValueError):
        DifferentialChassis("c", track_width=0.0)
    with pytest.raises(ValueError):
        DifferentialChassis("c", scrub_coeff=-0.1)


def test_diff_chassis_reset_returns_to_initial_pose():
    c = DifferentialChassis("c", x0=0.7, y0=0.3, theta0=-1.0)
    _run_diff(c, vL=0.2, vR=0.4, duration=0.3, dt=1e-3)
    assert (c.x, c.y, c.theta) != (0.7, 0.3, -1.0)
    c.reset()
    assert c.x == 0.7 and c.y == 0.3 and c.theta == -1.0
