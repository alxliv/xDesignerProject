# HANDOFF — xDesigner v0.1 → v0.2

Read this first, then `README.md` for the long form.

## What xDesigner is
A small Simulink-style Python simulation library for ≤ 50 cm / ≤ 2 kg robotic
devices. Block-based, scriptable, plays well with eventual AI control blocks.
Hardware target: **Raspberry Pi Pico 2W + Yahboom 520 motor** (12 V, 1:30
reduction, 333 RPM output, 11 PPR Hall encoder, 65 mm wheel).

## State of play (v0.1, working)
- Six built-in blocks: `SignalSource`, `PIDController`, `MotorDriver`,
  `DCMotor`, `Chassis`, `Encoder`.
- `Simulator` runs blocks in add-order, propagates after each step;
  feedback loops naturally carry a 1-sample delay (= real digital control).
- DC motor uses analytic electrical-step (no dt limit from L/R), forward Euler
  mechanical.
- Racing-car demo (`examples/racing_car.py`) closes a PID velocity loop and
  produces a 5-panel plot. Velocity tracks 0.8 m/s setpoint (top speed at
  this gearing is ~1.13 m/s, so the controller settles at ~72 % duty —
  physically consistent).

**Verify it still runs:**
```bash
pip install matplotlib
python examples/racing_car.py
```
Expect a console summary with final velocity ≈ 0.8 m/s and
`examples/racing_car.png` regenerated.

## Architectural choice that's worth revisiting first
v0.1 puts gearbox + reflected vehicle mass **inside** `DCMotor`, so the motor
owns the mechanical state and `Chassis` only computes resistive force. This
avoids algebraic loops but **couples motor and drivetrain in one block**.

The principled alternative is **bond-graph-style two-port mechanical
interfaces** — every mechanical element exposes an `(effort, flow)` port
(`(τ, ω)` rotational, `(F, v)` translational), and the simulator solves the
constraint network. Motor, gearbox, wheel, chassis become independently
swappable, and slip / clutch / belt blocks fit in naturally.

**Decision needed:** refactor to ports *before* v0.2, or wait until it hurts
(probably when we model a slipping tire)?

## v0.2 candidates
| | Direction | Roughly | Why |
|---|---|---|---|
| (a) | **2D dynamics** — Ackermann steering, lateral grip, x-y-θ chassis, track + lap timer | 2–3 days | Makes "racing" mean something |
| (b) | **AI controller block** — net-based policy with the same `setpoint/measurement → command` contract as PID; imitation-learn PID first, then RL | ~2 days | Validates the AI-as-block insertion point on the existing 1D loop |
| (c) | **HIL bridge** — same block graph runs in sim *or* on the Pico over USB serial; block interfaces become the firmware spec | ~3 days | "Designed on laptop, deployed to Pico" pipeline from day one |

Default recommendation: **(a) → (b) → (c)**. Flip if "design once, deploy to
hardware" is the higher-priority strategic goal — then (c) jumps to front.

## Open quick wins (~1 hr each, independent)
- `Battery` block — voltage sag vs. current (so duty-cycle math reflects real
  bus voltage).
- `DistanceSensor` / `BumpSensor` blocks with realistic noise + update rates.
- `pytest` suite covering at least the analytic electrical step and PID
  saturation/anti-windup.
- Optional sim configuration via YAML (so non-coders can tweak parameters).

## Repo layout
```
xdesigner_project/
├── README.md              ← long-form design + usage
├── HANDOFF.md             ← this file
├── xdesigner/
│   ├── block.py           ← Block, Port
│   ├── simulator.py       ← step loop, connections, probes
│   └── blocks/            ← six built-in blocks
└── examples/
    └── racing_car.py      ← the worked demo
```

## How to pick up from here
Don't write code yet. First, decide:
1. **Ports refactor** before v0.2, or defer?
2. **v0.2 direction** — (a), (b), or (c)?

Once those are answered, propose a concrete plan (files touched, new blocks,
example to demonstrate the result) before implementing. Keep the same
"one block per file, named ports, `step(t, dt)`" pattern v0.1 uses.
