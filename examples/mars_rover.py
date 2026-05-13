"""Mars-rover demo for xDesigner.

A small two-wheel skid-steer rover follows the same figure-8 track as
``racing_lap.py`` but with very different priorities:

* No Ackermann steering. Two independently driven front wheels, mixed
  in software by a ``DiffDriveMixer`` running on the Pico.
* Slow target speed (0.3 m/s). The score is **not** lap time.
* Two figures of merit:
    - **precision** : RMS lateral error on the centerline (smaller is better)
    - **economy**   : total electrical energy spent over the run
                      (smaller is better — both motors combined)

Block graph::

    ref_v ─► v_pid ─┐                                   ┌─► driver_L ─► motor_L ──┐
                    ├─► mixer ─► cmd_L (and cmd_R) ─►   │                         ├─► chassis
    pf ────────────►┘                                   └─► driver_R ─► motor_R ──┘
                                                                          │
    chassis.{x,y,θ,velocity} ─► pf and track ─► lap_timer                │
    chassis.F_resist_{L,R} ◄──────────────────────────────────────────────┘
    driver_{L,R}.voltage + motor_{L,R}.current ─► PowerMeter ─► energy
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..")))

import math

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from xDesigner import Simulator
from xDesigner.blocks import (
    DCMotor,
    DiffDriveMixer,
    DifferentialChassis,
    LapTimer,
    MotorDriver,
    PathFollower,
    PIDController,
    PowerMeter,
    SignalSource,
    Track,
)


TRACK_PATH = os.path.join(HERE, "tracks", "figure8.toml")
TRACK_WIDTH = 0.20            # m, lateral distance between the two wheels
TARGET_V = 0.30               # m/s, rover crawl speed


def build_rover_sim() -> Simulator:
    sim = Simulator(dt=5e-4)

    # --- reference + velocity loop ---------------------------------------
    ref_v = SignalSource("ref_v", waveform="step", amplitude=TARGET_V,
                         t_start=0.2)
    v_pid = PIDController(
        "v_pid",
        kp=0.9, ki=3.0, kd=0.01,
        out_min=-1.0, out_max=1.0,
        deriv_filter_tau=5e-3,
    )

    # --- track-following controller --------------------------------------
    track = Track.from_toml(TRACK_PATH, name="track")
    pf = PathFollower(
        "pf",
        track=track,
        # For a diff-drive rover these are tuning knobs, not physical
        # quantities. ``wheelbase`` scales the geometric δ; ``delta_max``
        # divides the normalised output. Lookahead must be SHORT or the
        # rover lags the centerline through the figure-8 loops.
        wheelbase=TRACK_WIDTH,
        delta_max=math.radians(40.0),
        lookahead_k=0.20,
        lookahead_min=0.10,
        lookahead_max=0.35,
    )

    # --- balancer / mixer + drivers + motors -----------------------------
    mixer = DiffDriveMixer("mixer", mixing_gain=0.9)
    driver_L = MotorDriver("driver_L", vbus=12.0)
    driver_R = MotorDriver("driver_R", vbus=12.0)
    # Each motor accelerates half the rover. Yahboom 520 same as before.
    motor_kw = dict(
        R=2.0, L=1.0e-3, ke=0.0115, kt=0.0115,
        J_rotor=1.0e-5, b=1.0e-6,
        gear_ratio=30.0, efficiency=0.85,
        wheel_radius=0.0325,
        reflected_mass=1.0,        # = mass / 2 (rover mass 2 kg)
    )
    motor_L = DCMotor("motor_L", **motor_kw)
    motor_R = DCMotor("motor_R", **motor_kw)

    # --- chassis ----------------------------------------------------------
    chassis = DifferentialChassis(
        "chassis",
        mass=2.0,
        track_width=TRACK_WIDTH,
        Crr=0.04, Cd=0.8, frontal_area=0.020,
        scrub_coeff=0.05,
        x0=1.2, y0=0.0, theta0=math.pi / 2,
    )

    # --- power meters and lap timer --------------------------------------
    pm_L = PowerMeter("pm_L")
    pm_R = PowerMeter("pm_R")
    lap = LapTimer("lap", track_length=track.length, min_lap_time=4.0)

    # add in dataflow order
    for b in (ref_v, v_pid, mixer,
              driver_L, driver_R, motor_L, motor_R,
              chassis, track, pf,
              pm_L, pm_R, lap):
        sim.add(b)

    # --- wiring -----------------------------------------------------------
    # velocity loop (uses average chassis velocity)
    sim.connect(ref_v["value"], v_pid["setpoint"])
    sim.connect(chassis["velocity"], v_pid["measurement"])
    sim.connect(v_pid["command"], mixer["throttle"])
    sim.connect(pf["steering_command"], mixer["steering"])

    # mixer → drivers → motors
    sim.connect(mixer["command_left"],  driver_L["command"])
    sim.connect(mixer["command_right"], driver_R["command"])
    sim.connect(driver_L["voltage"], motor_L["voltage"])
    sim.connect(driver_R["voltage"], motor_R["voltage"])

    # motors → chassis (wheel speeds)
    sim.connect(motor_L["velocity"], chassis["velocity_left"])
    sim.connect(motor_R["velocity"], chassis["velocity_right"])
    # chassis → motors (per-wheel resistive force)
    sim.connect(chassis["F_resist_left"],  motor_L["F_resist"])
    sim.connect(chassis["F_resist_right"], motor_R["F_resist"])

    # pose → track + pf
    sim.connect(chassis["x"], track["x"])
    sim.connect(chassis["y"], track["y"])
    sim.connect(chassis["x"], pf["x"])
    sim.connect(chassis["y"], pf["y"])
    sim.connect(chassis["theta"], pf["theta"])
    sim.connect(chassis["velocity"], pf["v"])

    # power meters
    sim.connect(driver_L["voltage"], pm_L["voltage"])
    sim.connect(motor_L["current"],  pm_L["current"])
    sim.connect(driver_R["voltage"], pm_R["voltage"])
    sim.connect(motor_R["current"],  pm_R["current"])

    # lap timer
    sim.connect(track["s_progress"], lap["s_progress"])

    return sim, track


def main() -> None:
    sim, track = build_rover_sim()

    probes = [
        "ref_v.value",
        "chassis.velocity", "chassis.yaw_rate",
        "chassis.x", "chassis.y", "chassis.theta",
        "track.s_progress", "track.lateral_error",
        "pf.steering_command",
        "mixer.command_left", "mixer.command_right",
        "motor_L.current", "motor_R.current",
        "pm_L.power", "pm_R.power",
        "pm_L.energy", "pm_R.energy",
        "lap.lap_count", "lap.last_lap_time", "lap.best_lap_time",
    ]

    duration = 60.0
    t, log = sim.run(duration=duration, probes=probes)

    # ---- score sheet -----------------------------------------------------
    lap_count = int(log["lap.lap_count"][-1])
    last_lap = log["lap.last_lap_time"][-1]
    best_lap = log["lap.best_lap_time"][-1]
    lateral = log["track.lateral_error"]
    rms_lat = math.sqrt(sum(e * e for e in lateral) / len(lateral))
    peak_lat = max(abs(e) for e in lateral)
    energy_L = log["pm_L.energy"][-1]
    energy_R = log["pm_R.energy"][-1]
    total_energy = energy_L + energy_R
    distance = sum(abs(v) * sim.dt for v in log["chassis.velocity"])
    energy_per_metre = total_energy / distance if distance > 0 else float("nan")

    print(f"Simulated {t[-1]:.2f} s with dt = {sim.dt*1000:.2f} ms "
          f"({len(t)} steps).")
    print(f"Track length      : {track.length:.3f} m")
    print(f"Laps completed    : {lap_count}")
    print(f"Last / best lap   : {last_lap:5.2f} s  /  {best_lap:5.2f} s")
    print(f"Distance travelled: {distance:6.3f} m")
    print("--- precision -----------------------------------------------")
    print(f"Lateral error     : RMS {rms_lat*1000:5.1f} mm,  "
          f"peak {peak_lat*1000:5.1f} mm")
    print("--- economy -------------------------------------------------")
    print(f"Energy (motor L)  : {energy_L:6.2f} J")
    print(f"Energy (motor R)  : {energy_R:6.2f} J")
    print(f"Total energy      : {total_energy:6.2f} J")
    print(f"Energy per metre  : {energy_per_metre:6.3f} J/m")

    # ---- plot ------------------------------------------------------------
    fig = plt.figure(figsize=(11, 10))
    gs = fig.add_gridspec(3, 2, height_ratios=[1.4, 1.0, 1.0])
    ax_xy = fig.add_subplot(gs[0, :])
    ax_v = fig.add_subplot(gs[1, 0])
    ax_lat = fig.add_subplot(gs[1, 1])
    ax_pow = fig.add_subplot(gs[2, 0])
    ax_cmd = fig.add_subplot(gs[2, 1])

    # XY trajectory + track
    wp = track.waypoints + [track.waypoints[0]]
    ax_xy.plot([p[0] for p in wp], [p[1] for p in wp],
               "k--", lw=1.0, alpha=0.55, label="centerline")
    ax_xy.plot(log["chassis.x"], log["chassis.y"], lw=1.4, label="rover path")
    ax_xy.plot([log["chassis.x"][0]], [log["chassis.y"][0]],
               "go", ms=6, label="start")
    ax_xy.plot([log["chassis.x"][-1]], [log["chassis.y"][-1]],
               "r^", ms=6, label="end")
    ax_xy.set_aspect("equal")
    ax_xy.set_title(f"Rover lap (×{lap_count} laps · "
                    f"RMS {rms_lat*1000:.1f} mm · "
                    f"{total_energy:.1f} J total)")
    ax_xy.set_xlabel("x (m)")
    ax_xy.set_ylabel("y (m)")
    ax_xy.legend(loc="upper right", fontsize=8)
    ax_xy.grid(True, alpha=0.3)

    # Velocity tracking
    ax_v.plot(t, log["ref_v.value"], "k--", lw=1.0, label="setpoint")
    ax_v.plot(t, log["chassis.velocity"], lw=1.3, label="rover v (avg)")
    ax_v.set_xlabel("time (s)")
    ax_v.set_ylabel("velocity (m/s)")
    ax_v.set_title("Longitudinal velocity")
    ax_v.legend(loc="lower right", fontsize=8)
    ax_v.grid(True, alpha=0.3)

    # Lateral error
    ax_lat.plot(t, [e * 1000 for e in log["track.lateral_error"]], lw=1.0)
    ax_lat.axhline(0, color="k", lw=0.5)
    ax_lat.set_xlabel("time (s)")
    ax_lat.set_ylabel("lateral error (mm)")
    ax_lat.set_title(f"Lateral error (RMS {rms_lat*1000:.1f} mm)")
    ax_lat.grid(True, alpha=0.3)

    # Power + cumulative energy (twin axis)
    ax_pow.plot(t, log["pm_L.power"], lw=0.7, alpha=0.6, label="P_L (W)")
    ax_pow.plot(t, log["pm_R.power"], lw=0.7, alpha=0.6, label="P_R (W)")
    ax_pow.set_xlabel("time (s)")
    ax_pow.set_ylabel("power (W)")
    ax_pow.set_title("Per-motor electrical power")
    ax_e = ax_pow.twinx()
    total_energy_curve = [a + b for a, b in zip(log["pm_L.energy"],
                                                log["pm_R.energy"])]
    ax_e.plot(t, total_energy_curve, "k-", lw=1.2, label="∑ energy (J)")
    ax_e.set_ylabel("energy (J)")
    # combine legends
    lines1, labels1 = ax_pow.get_legend_handles_labels()
    lines2, labels2 = ax_e.get_legend_handles_labels()
    ax_pow.legend(lines1 + lines2, labels1 + labels2,
                  loc="upper right", fontsize=8)
    ax_pow.grid(True, alpha=0.3)

    # Mixer commands
    ax_cmd.plot(t, log["mixer.command_left"], lw=1.0, label="cmd L")
    ax_cmd.plot(t, log["mixer.command_right"], lw=1.0, label="cmd R")
    ax_cmd.plot(t, log["pf.steering_command"], lw=0.6, alpha=0.5,
                label="steering (pre-mix)")
    ax_cmd.set_xlabel("time (s)")
    ax_cmd.set_ylabel("command (−1..+1)")
    ax_cmd.set_title("Mixer output to each motor")
    ax_cmd.legend(loc="lower right", fontsize=8)
    ax_cmd.grid(True, alpha=0.3)

    fig.suptitle("xDesigner — Mars-rover (skid-steer, precision + economy)",
                 y=0.995)
    fig.tight_layout()
    out_path = os.path.join(HERE, "mars_rover.png")
    fig.savefig(out_path, dpi=110)
    print(f"Plot saved to {out_path}")


if __name__ == "__main__":
    main()
