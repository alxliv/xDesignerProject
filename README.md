# xDesigner

A tiny Simulink-style simulation platform for small robotic devices
(≤ 50 cm, ≤ 2 kg). Written in pure Python with matplotlib for plots.
The first deliverable target is a small racing car driven by a
Yahboom 520 motor + Raspberry Pi Pico 2W.

This is **v0.1** — the bones, not the body. It defines the block
abstraction, a step-based simulator, six standard blocks, and one
worked example (PID velocity loop on a 1D car).

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

| Block            | Inputs                          | Outputs                                                            |
|------------------|---------------------------------|--------------------------------------------------------------------|
| `SignalSource`   | —                               | `value`                                                            |
| `PIDController`  | `setpoint`, `measurement`       | `command`                                                          |
| `MotorDriver`    | `command`                       | `voltage`                                                          |
| `DCMotor`        | `voltage`, `F_resist`           | `omega_motor`, `omega_out`, `current`, `torque`, `velocity`        |
| `Chassis`        | `velocity`                      | `F_resist`, `position`                                             |
| `Encoder`        | `omega`                         | `count`, `omega_measured`                                          |

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
python examples/racing_car.py
```

You'll get a console summary and `examples/racing_car.png`.

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
├── xdesigner/
│   ├── block.py            # Block, Port
│   ├── simulator.py        # Simulator (step loop, connections, probes)
│   └── blocks/
│       ├── signal_source.py
│       ├── pid.py
│       ├── motor_driver.py
│       ├── dc_motor.py
│       ├── chassis.py
│       └── encoder.py
└── examples/
    └── racing_car.py
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

1. **More signal sources / inputs** — joystick / log-file replay
   block for closed-loop sim from recorded data.
2. **2D chassis** — Ackermann steering, lateral dynamics, so the car
   can actually race a track. Adds a `Steering` block and an `(x, y, θ)`
   chassis.
3. **Sensor blocks** — IMU, distance sensor (HC-SR04 / ToF), line
   sensor — each with realistic noise and update rate.
4. **Reference tracker / track model** — a road-centerline + lap
   timer.
5. **AI controller block** — a wrapper around an MLP or small RL
   policy that exposes the same `setpoint, measurement → command`
   interface as PID.
6. **Hardware-in-the-loop bridge** — same block graph drives a
   simulator *or* a Pico over USB serial. The block descriptions
   become the firmware spec.
7. **Optional GUI** — block-diagram editor on top of the existing
   API. Out of scope for v0.1.

Open question for you: what should drive priorities for v0.2 — 2D
dynamics so the car can corner, or the AI-controller block on the
existing 1D loop?
