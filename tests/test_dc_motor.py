"""DCMotor tests: analytic electrical step, steady state, no-load top speed."""
import math

import pytest

from xDesigner.blocks import DCMotor


def _yahboom():
    """Defaults that match the racing-car demo."""
    return DCMotor("m",
                   R=2.0, L=1.0e-3, ke=0.0115, kt=0.0115,
                   J_rotor=1.0e-5, b=1.0e-6,
                   gear_ratio=30.0, efficiency=0.85,
                   wheel_radius=0.0325,
                   reflected_mass=1.0)


def test_electrical_step_locked_rotor_reaches_short_circuit_current():
    """With omega held at 0 (locked rotor) and 12 V applied, the analytic
    electrical update must asymptote the current to V/R = 6 A even when
    dt >> L/R (the analytic step's whole point: no stiff-ODE blowup)."""
    m = _yahboom()
    m.reset()
    m.inputs["voltage"].value = 12.0
    m.inputs["F_resist"].value = 0.0
    # Lock the rotor manually each step so we test the electrical loop alone.
    # Use dt = 10 ms — 20× the L/R time constant of 0.5 ms.
    for i in range(50):
        m.omega_motor = 0.0    # locked rotor
        m.step(t=i * 0.01, dt=0.01)
    assert m.i == pytest.approx(6.0, rel=1e-3)


def test_electrical_step_robust_to_large_dt():
    """Analytic step must NOT diverge when dt is many time constants —
    forward Euler on this equation would explode."""
    m = _yahboom()
    m.reset()
    m.inputs["voltage"].value = 12.0
    m.inputs["F_resist"].value = 0.0
    # dt = 1 s, L/R = 0.5 ms — that's 2000 time constants. Analytic
    # step jumps essentially straight to steady state.
    m.omega_motor = 0.0
    m.step(t=0.0, dt=1.0)
    assert m.i == pytest.approx(6.0, rel=1e-6)
    assert math.isfinite(m.i)


def test_no_load_top_speed_matches_back_emf_balance():
    """At no load (F_resist = 0, no internal friction), steady-state
    motor omega satisfies V = ke * omega → omega = V/ke.
    The vehicle-velocity output then is omega * r / N."""
    m = _yahboom()
    m.reset()
    m.b = 0.0   # remove internal friction so the balance is clean
    m.inputs["voltage"].value = 12.0
    m.inputs["F_resist"].value = 0.0
    # Long simulation to let mechanical state settle.
    for i in range(200000):
        m.step(t=i * 1e-4, dt=1e-4)
    expected_omega = 12.0 / 0.0115
    assert m.omega_motor == pytest.approx(expected_omega, rel=5e-3)
    expected_v = expected_omega * 0.0325 / 30.0
    assert m.outputs["velocity"].value == pytest.approx(expected_v, rel=5e-3)


def test_zero_voltage_zero_load_decays_to_rest():
    m = _yahboom()
    m.reset()
    m.omega_motor = 100.0
    m.i = 0.5
    m.inputs["voltage"].value = 0.0
    m.inputs["F_resist"].value = 0.0
    for i in range(200000):
        m.step(t=i * 1e-4, dt=1e-4)
    assert m.omega_motor == pytest.approx(0.0, abs=0.5)
    assert m.i == pytest.approx(0.0, abs=1e-3)


def test_reset_clears_state():
    m = _yahboom()
    m.inputs["voltage"].value = 12.0
    m.step(0.0, 1e-3)
    assert m.i != 0.0 or m.omega_motor != 0.0
    m.reset()
    assert m.i == 0.0 and m.omega_motor == 0.0


def test_validates_gear_ratio_and_efficiency():
    with pytest.raises(ValueError):
        DCMotor("bad", gear_ratio=0.0)
    with pytest.raises(ValueError):
        DCMotor("bad", efficiency=0.0)
    with pytest.raises(ValueError):
        DCMotor("bad", efficiency=1.5)


def test_velocity_output_consistent_with_gear_ratio():
    """For any omega_motor, velocity = omega * r / N exactly."""
    m = _yahboom()
    m.reset()
    m.omega_motor = 90.0       # arbitrary
    m.inputs["voltage"].value = 0.0
    m.inputs["F_resist"].value = 0.0
    m.step(0.0, 1e-9)          # tiny dt; outputs are recomputed
    assert m.outputs["velocity"].value == pytest.approx(
        m.omega_motor * 0.0325 / 30.0, rel=1e-6
    )
    assert m.outputs["omega_out"].value == pytest.approx(
        m.omega_motor / 30.0, rel=1e-6
    )
