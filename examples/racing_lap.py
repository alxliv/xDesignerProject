"""Racing-lap demo for xDesigner v0.2.

A small car with kinematic-bicycle steering drives a figure-8 track,
guided by a pure-pursuit path follower. A velocity PID maintains the
target speed; the lap timer records each crossing of the start/finish
line at s = 0 on the centerline.

Block graph::

    ref_v ─► v_pid ──► driver ──► motor ──► chassis2d ─┬─► track ──► lap_timer
                            ▲                          │     │
                            │                          │     ├─► lateral_error
                            │                          │     │
                            │                          ▼     ▼
                            │                       path_follower ──► steering
                            └─ F_resist ◄────── chassis2d ◄───────── delta ─┘
                            (motor.velocity ─► v_pid.measurement)
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
    Chassis2D,
    DCMotor,
    LapTimer,
    MotorDriver,
    PathFollower,
    PIDController,
    SignalSource,
    Steering,
    Track,
)


TRACK_PATH = os.path.join(HERE, "tracks", "figure8.toml")
WHEELBASE = 0.15  # m
DELTA_MAX = math.radians(30.0)


def build_lap_sim() -> Simulator:
    sim = Simulator(dt=5e-4)

    # --- reference and controllers ---------------------------------------
    ref_v = SignalSource("ref_v", waveform="step", amplitude=0.7, t_start=0.2)
    v_pid = PIDController(
        "v_pid",
        kp=0.8, ki=4.0, kd=0.02,
        out_min=-1.0, out_max=1.0,
        deriv_filter_tau=5e-3,
    )

    # --- powertrain (same Yahboom 520 model as the v0.1 demo) -------------
    driver = MotorDriver("driver", vbus=12.0)
    motor = DCMotor(
        "motor",
        R=2.0, L=1.0e-3, ke=0.0115, kt=0.0115,
        J_rotor=1.0e-5, b=1.0e-6,
        gear_ratio=30.0, efficiency=0.85,
        wheel_radius=0.0325,
        reflected_mass=1.0,
    )

    # --- 2D chassis, track, lap timer ------------------------------------
    chassis = Chassis2D(
        "chassis",
        mass=1.0,
        wheelbase=WHEELBASE,
        Crr=0.015, Cd=0.6, frontal_area=0.012,
        x0=1.2, y0=0.0, theta0=math.pi / 2,  # on the lemniscate at t=0
    )
    track = Track.from_toml(TRACK_PATH, name="track")
    lap = LapTimer("lap", track_length=track.length, min_lap_time=2.0)

    # --- steering controller (pure pursuit) + actuator -------------------
    pf = PathFollower(
        "pf",
        track=track,
        wheelbase=WHEELBASE,
        delta_max=DELTA_MAX,
        lookahead_k=0.25,
        lookahead_min=0.18,
        lookahead_max=0.60,
    )
    steering = Steering("steering", delta_max=DELTA_MAX, tau=0.04)

    # add in dataflow order
    for b in (ref_v, v_pid, driver, motor, chassis, track, pf, steering, lap):
        sim.add(b)

    # --- wiring -----------------------------------------------------------
    # velocity loop
    sim.connect(ref_v["value"], v_pid["setpoint"])
    sim.connect(motor["velocity"], v_pid["measurement"])
    sim.connect(v_pid["command"], driver["command"])
    sim.connect(driver["voltage"], motor["voltage"])
    sim.connect(chassis["F_resist"], motor["F_resist"])
    sim.connect(motor["velocity"], chassis["velocity"])

    # pose → track → lap timer
    sim.connect(chassis["x"], track["x"])
    sim.connect(chassis["y"], track["y"])
    sim.connect(track["s_progress"], lap["s_progress"])

    # pose + speed → pure pursuit → steering → chassis
    sim.connect(chassis["x"], pf["x"])
    sim.connect(chassis["y"], pf["y"])
    sim.connect(chassis["theta"], pf["theta"])
    sim.connect(motor["velocity"], pf["v"])
    sim.connect(pf["steering_command"], steering["command"])
    sim.connect(steering["delta"], chassis["delta"])

    return sim, track


def main() -> None:
    sim, track = build_lap_sim()

    probes = [
        "ref_v.value",
        "motor.velocity",
        "chassis.x",
        "chassis.y",
        "chassis.theta",
        "track.s_progress",
        "track.lateral_error",
        "track.heading_ref",
        "pf.steering_command",
        "steering.delta",
        "lap.lap_count",
        "lap.last_lap_time",
        "lap.best_lap_time",
        "lap.current_lap_time",
    ]

    duration = 30.0
    t, log = sim.run(duration=duration, probes=probes)

    # ---- summary ---------------------------------------------------------
    lap_count = int(log["lap.lap_count"][-1])
    last_lap = log["lap.last_lap_time"][-1]
    best_lap = log["lap.best_lap_time"][-1]
    lateral = log["track.lateral_error"]
    rms_lat = math.sqrt(sum(e * e for e in lateral) / len(lateral))
    peak_lat = max(abs(e) for e in lateral)
    peak_delta_deg = math.degrees(max(abs(d) for d in log["steering.delta"]))
    final_v = log["motor.velocity"][-1]
    setpoint = log["ref_v.value"][-1]

    print(f"Simulated {t[-1]:.2f} s with dt = {sim.dt*1000:.2f} ms "
          f"({len(t)} steps).")
    print(f"Track length     : {track.length:.3f} m")
    print(f"Final velocity   : {final_v:5.3f} m/s   (target {setpoint:.3f} m/s)")
    print(f"Laps completed   : {lap_count}")
    print(f"Last lap time    : {last_lap:5.3f} s")
    print(f"Best lap time    : {best_lap:5.3f} s")
    print(f"Lateral error    : RMS {rms_lat*1000:5.1f} mm,  peak {peak_lat*1000:5.1f} mm")
    print(f"Peak steering    : {peak_delta_deg:5.2f}°")

    # ---- plot ------------------------------------------------------------
    fig = plt.figure(figsize=(11, 9))
    gs = fig.add_gridspec(3, 2, height_ratios=[1.4, 1.0, 1.0])
    ax_xy = fig.add_subplot(gs[0, :])
    ax_v = fig.add_subplot(gs[1, 0])
    ax_lat = fig.add_subplot(gs[1, 1])
    ax_delta = fig.add_subplot(gs[2, 0])
    ax_lap = fig.add_subplot(gs[2, 1])

    # XY trajectory + track centerline
    wp = track.waypoints + [track.waypoints[0]]  # close polygon for plotting
    ax_xy.plot([p[0] for p in wp], [p[1] for p in wp],
               "k--", lw=1.0, alpha=0.55, label="centerline")
    ax_xy.plot(log["chassis.x"], log["chassis.y"], lw=1.4, label="car path")
    ax_xy.plot([log["chassis.x"][0]], [log["chassis.y"][0]],
               "go", ms=6, label="start")
    ax_xy.plot([log["chassis.x"][-1]], [log["chassis.y"][-1]],
               "r^", ms=6, label="end")
    ax_xy.set_aspect("equal")
    ax_xy.set_title(f"Figure-8 lap (×{lap_count} laps, best {best_lap:.2f}s)")
    ax_xy.set_xlabel("x (m)")
    ax_xy.set_ylabel("y (m)")
    ax_xy.legend(loc="upper right", fontsize=8)
    ax_xy.grid(True, alpha=0.3)

    # Velocity tracking
    ax_v.plot(t, log["ref_v.value"], "k--", lw=1.0, label="setpoint")
    ax_v.plot(t, log["motor.velocity"], lw=1.3, label="actual v")
    ax_v.set_xlabel("time (s)")
    ax_v.set_ylabel("velocity (m/s)")
    ax_v.set_title("Longitudinal velocity")
    ax_v.legend(loc="lower right", fontsize=8)
    ax_v.grid(True, alpha=0.3)

    # Lateral error
    ax_lat.plot(t, [e * 1000 for e in log["track.lateral_error"]],
                lw=1.0, label="lateral error")
    ax_lat.axhline(0, color="k", lw=0.5)
    ax_lat.set_xlabel("time (s)")
    ax_lat.set_ylabel("lateral error (mm)")
    ax_lat.set_title(f"Lateral error (RMS {rms_lat*1000:.1f} mm)")
    ax_lat.grid(True, alpha=0.3)

    # Steering command + delta
    ax_delta.plot(t, log["pf.steering_command"], lw=0.8, alpha=0.7,
                  label="cmd (−1..+1)")
    ax_delta.plot(t, [math.degrees(d) / math.degrees(DELTA_MAX)
                      for d in log["steering.delta"]],
                  lw=1.1, label="δ / δ_max")
    ax_delta.set_xlabel("time (s)")
    ax_delta.set_ylabel("normalised")
    ax_delta.set_title("Steering command and actuator response")
    ax_delta.legend(loc="lower right", fontsize=8)
    ax_delta.grid(True, alpha=0.3)

    # Lap timer trace
    ax_lap.plot(t, log["lap.current_lap_time"], lw=1.0, label="current lap")
    ax_lap.plot(t, log["lap.last_lap_time"], lw=1.0, label="last lap")
    ax_lap.plot(t, log["lap.best_lap_time"], lw=1.0, label="best lap")
    ax_lap.set_xlabel("time (s)")
    ax_lap.set_ylabel("seconds")
    ax_lap.set_title("Lap timer")
    ax_lap.legend(loc="upper right", fontsize=8)
    ax_lap.grid(True, alpha=0.3)

    fig.suptitle("xDesigner v0.2 — figure-8 with pure-pursuit steering", y=0.995)
    fig.tight_layout()
    out_path = os.path.join(HERE, "racing_lap.png")
    fig.savefig(out_path, dpi=110)
    print(f"Plot saved to {out_path}")


if __name__ == "__main__":
    main()
