# xDesigner

A tiny Simulink-style simulation platform for small robotic devices
(≤ 50 cm, ≤ 2 kg). Written in pure Python with matplotlib for plots.
The first deliverable target is a small racing car driven by a
Yahboom 520 motor + Raspberry Pi Pico 2W.

This is **v0.2**. v0.1 was the bones — block abstraction, step-based
simulator, six 1D blocks, and a velocity-loop demo. v0.2 adds 2D
dynamics: a kinematic-bicycle chassis, a steering actuator, a
TOML-defined track with centerline projection, a pure-pursuit path
follower, and a lap timer. The new demo drives a figure-8 closed track.

---

## Why it exists

Simulink is great but heavy. For our scale of device (a few sensors,
one or two motors, a microcontroller) we want:

* an open, scriptable representation we can put under version control;
* the same blocks that map cleanly onto Pico firmware later;
* a place to plug in AI elements (a neural policy is just another block
  with `setpoint, measurement → command`);
* fast iteration: change a parameter, replot in under a second.

## The mental model

Each `Block` has named **input ports** and **output ports** and a
`step(t, dt)` method. The `Simulator` runs the blocks in the order
they were added, copying signals along the connections after each
block updates. Feedback signals (an output of a later block flowing
back to an earlier block) naturally pick up a one-sample delay — which
is exactly how real digital control loops behave.

```
SignalSource ─► PIDController ─► MotorDriver ─► DCMotor ─► Chassis
                       ▲                           │           │
                       │                           ▼           │
                       └────────── motor.velocity ◄────────────┘
                                                   ▲
                                          (chassis.F_resist
                                           feeds back to motor)
```

## Built-in blocks

v0.1 (1D powertrain):

| Block            | Inputs                          | Outputs                                                            |
|------------------|---------------------------------|--------------------------------------------------------------------|
| `SignalSource`   | —                               | `value`                                                            |
| `PIDController`  | `setpoint`, `measurement`       | `command`                                                          |
| `MotorDriver`    | `command`                       | `voltage`                                                          |
| `DCMotor`        | `voltage`, `F_resist`           | `omega_motor`, `omega_out`, `current`, `torque`, `velocity`        |
| `Chassis`        | `velocity`                      | `F_resist`, `position`                                             |
| `Encoder`        | `omega`                         | `count`, `omega_measured`                                          |

v0.2 (2D dynamics — Ackermann racing car):

| Block            | Inputs                          | Outputs                                                            |
|------------------|---------------------------------|--------------------------------------------------------------------|
| `Chassis2D`      | `velocity`, `delta`             | `F_resist`, `x`, `y`, `theta`, `yaw_rate`                          |
| `Steering`       | `command`                       | `delta`                                                            |
| `Track`          | `x`, `y`                        | `s_progress`, `lateral_error`, `heading_ref`, `curvature`          |
| `PathFollower`   | `x`, `y`, `theta`, `v`          | `steering_command`                                                 |
| `LapTimer`       | `s_progress`                    | `lap_count`, `current_lap_time`, `last_lap_time`, `best_lap_time`  |

v0.2 (skid-steer rover):

| Block                  | Inputs                                  | Outputs                                                                  |
|------------------------|-----------------------------------------|--------------------------------------------------------------------------|
| `DifferentialChassis`  | `velocity_left`, `velocity_right`       | `F_resist_left`, `F_resist_right`, `velocity`, `yaw_rate`, `x`, `y`, `theta` |
| `DiffDriveMixer`       | `throttle`, `steering`                  | `command_left`, `command_right`                                          |
| `PowerMeter`           | `voltage`, `current`                    | `power`, `energy`                                                        |

`Chassis2D` coexists with the 1D `Chassis`; `DifferentialChassis` is a
parallel option for skid-steer vehicles — pick whichever fits the
demo. `Track` is also a regular Python object exposing
`from_toml(path)`, `length`, `project(x, y)`, and `lookahead_point(x, y, d)`
so `PathFollower` (and your own blocks) can query it imperatively.

Defaults for `DCMotor` match a Yahboom 520 at 12 V with 1:30 reduction
and a 65 mm wheel — change them in your script to model the variant
you actually have.

> A note on the architecture: `DCMotor` embeds the gearbox and the
> reflected vehicle mass, so it owns the mechanical state. `Chassis`
> only computes resistive force and tracks position. That keeps the
> coupled rigid-body equations of motion in one place and avoids
> algebraic loops at this stage. A future bond-graph-style ports
> layer can lift this restriction.

## Quickstart

```bash
pip install matplotlib
python examples/racing_car.py    # 1D velocity-loop demo (v0.1)
python examples/racing_lap.py    # 2D figure-8 with pure-pursuit (v0.2)
python examples/mars_rover.py    # skid-steer rover, scored on precision + energy
```

You'll get a console summary and a `*.png` next to each script.
`racing_lap.py` optimises for lap time; `mars_rover.py` deliberately
flips the objective — slow target speed, the score sheet reports
RMS lateral error (mm) and total electrical energy (J).

### Defining a track

Tracks are TOML files under `examples/tracks/`:

```toml
name = "figure-8"
closed = true
waypoints = [
    [1.199, 0.063],
    [1.190, 0.187],
    ...
]
```

`examples/tracks/_generate.py` produces `figure8.toml` by sampling a
Bernoulli lemniscate; tweak the parameters and re-run to get a different
shape, or hand-write a TOML for a custom course.

### Minimal script

```python
from xDesigner import Simulator
from xDesigner.blocks import (
    SignalSource, PIDController, MotorDriver, DCMotor, Chassis, Encoder
)

sim = Simulator(dt=5e-4)

ref     = sim.add(SignalSource("ref", waveform="step", amplitude=0.8, t_start=0.2))
pid     = sim.add(PIDController("pid", kp=0.8, ki=4.0, kd=0.02))
driver  = sim.add(MotorDriver("driver", vbus=12.0))
motor   = sim.add(DCMotor("motor", gear_ratio=30.0, wheel_radius=0.0325,
                          reflected_mass=1.0))
chassis = sim.add(Chassis("chassis", mass=1.0))
enc     = sim.add(Encoder("enc", ppr=11))

sim.connect(ref["value"],            pid["setpoint"])
sim.connect(motor["velocity"],       pid["measurement"])
sim.connect(pid["command"],          driver["command"])
sim.connect(driver["voltage"],       motor["voltage"])
sim.connect(chassis["F_resist"],     motor["F_resist"])
sim.connect(motor["velocity"],       chassis["velocity"])
sim.connect(motor["omega_motor"],    enc["omega"])

t, log = sim.run(duration=2.0, probes=["ref.value", "motor.velocity"])
```

### Adding your own block

```python
from xDesigner import Block

class MyController(Block):
    def __init__(self, name, gain=1.0):
        super().__init__(name)
        self.gain = gain
        self.add_input("error")
        self.add_output("command")
    def step(self, t, dt):
        self.outputs["command"].value = self.gain * self.inputs["error"].value
```

Any `Block` works. A `NeuralController` block wrapping a small policy
network is the same recipe — input ports for state, output port for
command, model weights as parameters. That is the natural insertion
point for the AI elements you're planning.

## Repo layout

```
xdesigner_project/
├── xDesigner/
│   ├── block.py            # Block, Port
│   ├── simulator.py        # Simulator (step loop, connections, probes)
│   └── blocks/
│       ├── signal_source.py
│       ├── pid.py
│       ├── motor_driver.py
│       ├── dc_motor.py
│       ├── chassis.py
│       ├── encoder.py
│       ├── chassis2d.py        # v0.2 (Ackermann)
│       ├── steering.py         # v0.2
│       ├── track.py            # v0.2
│       ├── path_follower.py    # v0.2 (pure pursuit)
│       ├── lap_timer.py        # v0.2
│       ├── differential_chassis.py  # v0.2 (skid-steer)
│       ├── diff_drive_mixer.py      # v0.2
│       └── power_meter.py           # v0.2
└── examples/
    ├── racing_car.py           # v0.1 demo
    ├── racing_lap.py           # v0.2 demo (Ackermann racing car)
    ├── mars_rover.py           # v0.2 demo (skid-steer rover)
    └── tracks/
        ├── _generate.py        # writes figure8.toml
        └── figure8.toml
```

## Sanity check from the demo run

* Top speed at 12 V, 1:30, 65 mm wheel: **1.13 m/s** (computed
  from `12 / ke / N * r`). Asking for anything above that → controller
  saturates. The demo targets 0.8 m/s, ~71 % of top speed, which lands
  at a steady command of ~0.72 — physically consistent.
* Encoder produces visibly quantized speed estimates at low rates
  (11 PPR × 4 = 44 counts/rev, sampled at 1 kHz). This matches what the
  Pico will actually see and is the right signal to design your
  filtering and PID gains against.

## What's next (in roughly increasing effort)

1. **Dynamic bicycle** — replace the kinematic `Chassis2D` with a
   lateral-grip model (slip angles, cornering stiffness) so tire limits
   bite. Drop-in: same ports.
2. **Sensor blocks** — IMU, distance sensor (HC-SR04 / ToF), line
   sensor — each with realistic noise and update rate.
3. **AI controller block** — a wrapper around an MLP or small RL
   policy that exposes the same `(x, y, theta, v) → steering_command`
   interface as `PathFollower`. Imitation-learn pure pursuit first,
   then RL on lap time.
4. **Hardware-in-the-loop bridge** — same block graph drives a
   simulator *or* a Pico over USB serial. The block descriptions
   become the firmware spec.
5. **Optional GUI** — block-diagram editor on top of the existing
   API.
