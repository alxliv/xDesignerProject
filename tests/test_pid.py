"""Tests for PIDController: saturation, anti-windup, derivative filtering."""
import pytest

from xDesigner.blocks import PIDController


def _drive(pid, setpoint, measurement, dt, n_steps):
    """Step the PID n times with constant setpoint/measurement."""
    pid.inputs["setpoint"].value = setpoint
    pid.inputs["measurement"].value = measurement
    for i in range(n_steps):
        pid.step(t=i * dt, dt=dt)
    return pid.outputs["command"].value


def test_proportional_only_no_integral_drift():
    pid = PIDController("p", kp=2.0, ki=0.0, kd=0.0)
    cmd = _drive(pid, setpoint=1.0, measurement=0.0, dt=1e-3, n_steps=10)
    # P-only: command = kp * error = 2 * 1 = 2 (then clamped to out_max=1).
    assert cmd == pytest.approx(1.0)


def test_integral_accumulates_unsaturated():
    pid = PIDController("i", kp=0.0, ki=1.0, kd=0.0,
                        out_min=-100.0, out_max=100.0)
    cmd = _drive(pid, setpoint=1.0, measurement=0.0, dt=1e-3, n_steps=500)
    # err=1 for 500 steps of 1ms → integral ≈ 0.5; command = ki*0.5 = 0.5.
    assert cmd == pytest.approx(0.5, abs=2e-3)


def test_anti_windup_holds_integral_during_saturation():
    """While the output is clamped AND the error pushes further into the
    clamp, the integral must not grow — that's the anti-windup guarantee."""
    pid = PIDController("aw", kp=0.0, ki=1.0, kd=0.0,
                        out_min=-1.0, out_max=1.0)
    # Force into positive saturation: err = +1 for 5 s.
    _drive(pid, setpoint=1.0, measurement=0.0, dt=1e-3, n_steps=5000)
    integral_at_saturation = pid._integral
    # Hold setpoint/measurement: another 5 s of err=+1 should NOT grow
    # the integral further (we're saturated and error pushes deeper).
    _drive(pid, setpoint=1.0, measurement=0.0, dt=1e-3, n_steps=5000)
    assert pid._integral == pytest.approx(integral_at_saturation, abs=1e-9)
    # Output remains clamped to out_max.
    assert pid.outputs["command"].value == pytest.approx(1.0)


def test_anti_windup_unwinds_on_error_reversal():
    """Saturated and pushing further? Hold the integral. Saturated but
    error reverses sign? Integrate normally — the integral must be
    able to come down."""
    pid = PIDController("rev", kp=0.0, ki=1.0, kd=0.0,
                        out_min=-1.0, out_max=1.0)
    _drive(pid, setpoint=1.0, measurement=0.0, dt=1e-3, n_steps=5000)
    saturated_int = pid._integral
    # Reverse the error; integral must decrease.
    _drive(pid, setpoint=0.0, measurement=1.0, dt=1e-3, n_steps=100)
    assert pid._integral < saturated_int


def test_reset_zeroes_state():
    pid = PIDController("r", kp=0.0, ki=1.0, kd=0.0,
                        out_min=-100.0, out_max=100.0)
    _drive(pid, setpoint=1.0, measurement=0.0, dt=1e-3, n_steps=20)
    assert pid._integral > 0.0
    pid.reset()
    assert pid._integral == 0.0
    assert pid._prev_error == 0.0
    assert pid._deriv == 0.0


def test_derivative_filter_smooths_step():
    """An unfiltered derivative on a measurement step would produce a
    huge spike; the filter must attenuate it."""
    dt = 1e-3
    pid_filt = PIDController("f", kp=0.0, ki=0.0, kd=1.0,
                             out_min=-1e6, out_max=1e6,
                             deriv_filter_tau=10e-3)
    pid_raw = PIDController("r", kp=0.0, ki=0.0, kd=1.0,
                            out_min=-1e6, out_max=1e6,
                            deriv_filter_tau=0.0)
    # Apply a step change in measurement at t=0.
    pid_filt.inputs["setpoint"].value = 0.0
    pid_raw.inputs["setpoint"].value = 0.0
    pid_filt.inputs["measurement"].value = 1.0
    pid_raw.inputs["measurement"].value = 1.0
    pid_filt.step(0.0, dt)
    pid_raw.step(0.0, dt)
    # Both see a huge first-step spike, but the filtered one applies
    # a low-pass with tau >> dt → attenuation factor dt/(tau+dt) ≈ 1/11.
    assert abs(pid_filt.outputs["command"].value) < abs(pid_raw.outputs["command"].value)
