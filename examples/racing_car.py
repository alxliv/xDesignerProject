"""Racing-car demo for xDesigner.

Wires up:

    SignalSource ─► PIDController ─► MotorDriver ─► DCMotor ─► Chassis
                            ▲                          │
                            │                          ▼
                            └─ Encoder ◄────── omega_motor
                            (measurement of vehicle velocity)

Defaults model a small car (~1 kg, 50 cm-class) driven by a single
Yahboom 520 motor with 1:30 reduction and a 65 mm wheel. The PID tries
to follow a velocity reference of 2 m/s (≈ 7.2 km/h).
"""
import os
import sys

# Allow running this file directly without installing the package.
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..")))

import matplotlib

matplotlib.use("Agg")  # headless-safe; remove to use the default GUI backend
import matplotlib.pyplot as plt

from xDesigner import Simulator
from xDesigner.blocks import (
    SignalSource,
    PIDController,
    MotorDriver,
    DCMotor,
    Chassis,
    Encoder,
)


def build_car_sim() -> Simulator:
    sim = Simulator(dt=5e-4)  # 0.5 ms step

    # --- Blocks (Yahboom 520-class, ~1 kg car, 65 mm wheel) ---------------
    # Note on top speed: at 12 V and 1:30 reduction, the no-load motor
    # output is ~34.9 rad/s, so the no-load vehicle speed is ~1.13 m/s.
    # Choose a setpoint well below that to see proper closed-loop tracking.
    ref = SignalSource(
        "ref",
        waveform="step",
        amplitude=0.8,     # target 0.8 m/s  (≈ 2.9 km/h)
        t_start=0.2,       # ramp up after 0.2 s of idle
    )

    pid = PIDController(
        "pid",
        kp=0.8,
        ki=4.0,
        kd=0.02,
        out_min=-1.0,
        out_max=1.0,
        deriv_filter_tau=5e-3,
    )

    driver = MotorDriver("driver", vbus=12.0)

    motor = DCMotor(
        "motor",
        R=2.0,
        L=1.0e-3,
        ke=0.0115,        # ~12V / 1046 rad/s no-load
        kt=0.0115,        # numerically equal to ke in SI
        J_rotor=1.0e-5,
        b=1.0e-6,
        gear_ratio=30.0,
        efficiency=0.85,
        wheel_radius=0.0325,
        reflected_mass=1.0,
    )

    chassis = Chassis(
        "chassis",
        mass=1.0,
        Crr=0.015,
        Cd=0.6,
        frontal_area=0.012,
    )

    encoder = Encoder("enc", ppr=11, quadrature=True, dt_sample=1e-3)

    # Order matters: add in dataflow order for forward propagation.
    sim.add(ref)
    sim.add(pid)
    sim.add(driver)
    sim.add(motor)
    sim.add(chassis)
    sim.add(encoder)

    # --- Wiring -----------------------------------------------------------
    sim.connect(ref["value"], pid["setpoint"])
    sim.connect(motor["velocity"], pid["measurement"])  # closed loop on velocity
    sim.connect(pid["command"], driver["command"])
    sim.connect(driver["voltage"], motor["voltage"])
    sim.connect(chassis["F_resist"], motor["F_resist"])
    sim.connect(motor["velocity"], chassis["velocity"])
    sim.connect(motor["omega_motor"], encoder["omega"])

    return sim


def main():
    sim = build_car_sim()

    probes = [
        "ref.value",
        "motor.velocity",
        "pid.command",
        "driver.voltage",
        "motor.current",
        "motor.omega_motor",
        "enc.count",
        "enc.omega_measured",
        "chassis.position",
        "chassis.F_resist",
    ]

    t, log = sim.run(duration=2.0, probes=probes)

    # ---- summary --------------------------------------------------------
    final_v = log["motor.velocity"][-1]
    final_pos = log["chassis.position"][-1]
    peak_current = max(abs(c) for c in log["motor.current"])
    final_count = log["enc.count"][-1]
    setpoint = log["ref.value"][-1]
    print(f"Simulated {t[-1]:.2f} s with dt = {sim.dt*1000:.2f} ms "
          f"({len(t)} steps).")
    print(f"Final velocity : {final_v:6.3f} m/s   (target {setpoint:.3f} m/s)")
    print(f"Final position : {final_pos:6.3f} m")
    print(f"Peak current   : {peak_current:6.3f} A")
    print(f"Encoder count  : {final_count:.0f} ticks  "
          f"({final_count / (4*11):.1f} motor revs, "
          f"{final_count / (4*11*30):.1f} wheel revs)")

    # ---- plot -----------------------------------------------------------
    fig, axes = plt.subplots(5, 1, figsize=(9, 11), sharex=True)

    # 1. Vehicle velocity tracking (m/s)
    enc_v = [w / 30.0 * 0.0325 for w in log["enc.omega_measured"]]
    axes[0].plot(t, log["ref.value"], "k--", lw=1.2, label="setpoint")
    axes[0].plot(t, log["motor.velocity"], lw=1.6, label="actual v")
    axes[0].plot(t, enc_v, lw=0.7, alpha=0.55, label="encoder estimate")
    axes[0].set_ylabel("velocity (m/s)")
    axes[0].set_title("Velocity tracking")
    axes[0].legend(loc="lower right", fontsize=8)
    axes[0].grid(True, alpha=0.3)

    # 2. Motor shaft angular velocity (rad/s) — true vs encoder-measured
    axes[1].plot(t, log["motor.omega_motor"], lw=1.4, label="ω motor (true)")
    axes[1].plot(t, log["enc.omega_measured"], lw=0.7, alpha=0.55,
                 label="ω motor (encoder)")
    axes[1].set_ylabel("ω motor (rad/s)")
    axes[1].set_title("Motor shaft speed (encoder noise visible)")
    axes[1].legend(loc="lower right", fontsize=8)
    axes[1].grid(True, alpha=0.3)

    # 3. Control signal & driver voltage
    axes[2].plot(t, log["pid.command"], label="PID command (−1..+1)")
    axes[2].plot(t, [v / 12.0 for v in log["driver.voltage"]],
                 lw=0.8, alpha=0.6, label="driver V / V_bus")
    axes[2].set_ylabel("control")
    axes[2].set_title("Controller output")
    axes[2].legend(loc="lower right", fontsize=8)
    axes[2].grid(True, alpha=0.3)

    # 4. Current and resistive force
    axes[3].plot(t, log["motor.current"], label="motor current (A)")
    axes[3].plot(t, log["chassis.F_resist"], lw=0.8, alpha=0.7,
                 label="F_resist (N)")
    axes[3].set_ylabel("A / N")
    axes[3].set_title("Motor current and load")
    axes[3].legend(loc="upper right", fontsize=8)
    axes[3].grid(True, alpha=0.3)

    # 5. Position
    axes[4].plot(t, log["chassis.position"], label="position (m)")
    axes[4].set_ylabel("position (m)")
    axes[4].set_xlabel("time (s)")
    axes[4].set_title("Distance travelled")
    axes[4].legend(loc="lower right", fontsize=8)
    axes[4].grid(True, alpha=0.3)

    fig.suptitle("xDesigner v0.1 — racing-car velocity control", y=0.995)
    fig.tight_layout()
    out_path = os.path.join(HERE, "racing_car.png")
    fig.savefig(out_path, dpi=110)
    print(f"Plot saved to {out_path}")


if __name__ == "__main__":
    main()
