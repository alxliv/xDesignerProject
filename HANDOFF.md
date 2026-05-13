# HANDOFF — xDesigner v0.2 → v0.3

Read this first, then `README.md` for the long form.

## What xDesigner is
A small Simulink-style Python simulation library for ≤ 50 cm / ≤ 2 kg
robotic devices. Block-based, scriptable, plays well with eventual AI
control blocks. Hardware target: **Raspberry Pi Pico 2W + Yahboom 520
motor** (12 V, 1:30 reduction, 333 RPM output, 11 PPR Hall encoder,
65 mm wheel).

## State of play (v0.2, working)

**v0.1 (untouched, still passes regression):** six 1D blocks
(`SignalSource`, `PIDController`, `MotorDriver`, `DCMotor`, `Chassis`,
`Encoder`) + `examples/racing_car.py`.

**v0.2 (new):** 2D dynamics on top of the same simulator.

* `Chassis2D` — kinematic-bicycle chassis (no slip), x/y/θ pose,
  same `F_resist` contract as 1D `Chassis` so motor wiring is identical.
* `Steering` — normalised command → steering angle, with saturation
  and first-order actuator lag.
* `Track` — closed centerline of (x, y) waypoints, loaded from TOML
  via `Track.from_toml(path)`. Projects the car onto the centerline
  each step and emits `s_progress`, `lateral_error`, `heading_ref`,
  `curvature`. Also exposes `lookahead_point(x, y, d)` for imperative
  callers (the path follower).
* `PathFollower` — geometric pure-pursuit steering controller. Holds
  a reference to a `Track` and produces a normalised steering command.
* `LapTimer` — wraps `s_progress` to count laps and track best/last
  lap times, with a `min_lap_time` guard against double-counting.

**Demo:** `examples/racing_lap.py` drives a Bernoulli-lemniscate
figure-8 (≈6.28 m perimeter) at 0.7 m/s. Acceptance numbers from the
latest run:

| | result |
|---|---|
| v0.1 `racing_car.py` regression | identical to v0.1 (final v ≈ 0.799 m/s) |
| laps in 30 s | 3 |
| best lap | 8.97 s |
| lateral error (RMS / peak) | 9.6 mm / 12.4 mm |
| peak steering | 21° (δ_max = 30°) |

**Verify it all runs:**
```bash
pip install matplotlib
python examples/racing_car.py    # v0.1 regression
python examples/racing_lap.py    # v0.2 demo
```
Outputs `racing_car.png` and `racing_lap.png` in `examples/`.

## Architectural choice still pending

v0.1 embedded gearbox + reflected vehicle mass inside `DCMotor`.
v0.2 left that alone — `Chassis2D` adds kinematics on top, motor still
owns longitudinal mechanical state. The principled
bond-graph-style two-port refactor is **still deferred**; v0.2's needs
didn't force it. The decision will come due when we add a dynamic
bicycle with tire slip / clutch / belt — then having a proper
`(τ, ω)` / `(F, v)` port abstraction stops being optional.

## v0.3 candidates
| | Direction | Roughly | Why |
|---|---|---|---|
| (a) | **Dynamic bicycle** — replace kinematic `Chassis2D` with slip-angle + cornering-stiffness model; same ports, drop-in | 2–3 days | Makes "racing limit" meaningful; pure pursuit will visibly understeer |
| (b) | **AI controller block** — net-based policy with same `(x, y, θ, v) → steering_command` contract as `PathFollower`; imitation-learn pure pursuit first, then RL on lap time | ~2 days | Validates AI-as-block insertion on the 2D loop |
| (c) | **HIL bridge** — same block graph runs in sim or on the Pico over USB serial; block interfaces become firmware spec | ~3 days | "Designed on laptop, deployed to Pico" pipeline |

Default recommendation: **(b) → (a) → (c)**. (b) is now the obvious
next step because pure pursuit is the baseline an AI block has to beat,
and 2D gives RL a non-trivial cost (lap time, lateral error). (a) is
worth doing before HIL only because a slip-aware sim is what makes the
AI policy non-trivial.

## Open quick wins (~1 hr each, independent)
- `Battery` block — voltage sag vs. current (so duty-cycle math reflects real bus voltage).
- `DistanceSensor` / `BumpSensor` blocks with realistic noise + update rates.
- `pytest` suite covering at least: analytic electrical step, PID saturation/anti-windup, `Track.project` near the figure-8 crossing, `LapTimer` wrap detection.
- More tracks: a fast oval or a road-course TOML to exercise different curvature regimes.

## Repo layout
```
xdesigner_project/
├── README.md
├── HANDOFF.md
├── xDesigner/
│   ├── block.py           ← Block, Port
│   ├── simulator.py       ← step loop, connections, probes
│   └── blocks/            ← 6 v0.1 blocks + 5 v0.2 blocks
└── examples/
    ├── racing_car.py      ← v0.1 demo (regression check)
    ├── racing_lap.py      ← v0.2 demo (figure-8 + pure pursuit)
    └── tracks/
        ├── _generate.py
        └── figure8.toml
```

## How to pick up from here
Don't write code yet. First, decide:
1. **Ports refactor** — defer again, or do it before (a) dynamic bicycle?
2. **v0.3 direction** — (a), (b), or (c)?

Once those are answered, propose a concrete plan (files touched,
new blocks, example to demonstrate the result) before implementing.
Keep the same "one block per file, named ports, `step(t, dt)`" pattern.
