# sim2sim-locomotion

Evaluate **one** locomotion policy across **multiple physics simulators** —
MuJoCo, PyBullet, Genesis, and Isaac Lab — and compare the results side by side.

This is the classic **sim-to-sim** robustness check that sits between training
and sim-to-real. If a policy behaves consistently across simulators with
different contact solvers, integrators, and actuator models, it is far more
likely to transfer to real hardware. Divergence between simulators flags a
policy that has overfit to one simulator's quirks.

The policy is loaded from **ONNX**, so the *exact same* `.onnx` file is fed to
every backend — no per-simulator reimplementation of the network.

---

## Why this design works

The hard part of sim-to-sim is **not** wiring up simulators. It is guaranteeing
that every backend hands the policy a **bit-for-bit identical observation
vector** and applies actions through an **identical control law**. So those two
things are centralized and the simulators are kept deliberately thin:

```
            ┌──────────────────────── runner (episode loop) ────────────────────────┐
 RobotState │  ObservationBuilder ── obs ──▶ Policy(.onnx) ── action ──▶ PDController │ torque
   ◀────────┤        (obs/)                  (policy/)                   (control/)   ├────────▶
            └─────────────────────────────────────────────────────────────────────-┘
                         ▲                                                    │
                         │  get_state()                          apply_torques()/step()
                         └────────────────  Simulator adapter  ──────────────┘
                                  (mujoco | pybullet | genesis | isaaclab)
```

- A neutral [`RobotState`](src/sim2sim/sim/state.py) is the only thing adapters
  produce. Every adapter reports joints in one **canonical order** (a
  joint-order mismatch across simulators is the #1 sim-to-sim bug).
- [`ObservationBuilder`](src/sim2sim/obs/observation.py) assembles the policy
  input from `RobotState` + command in a **config-driven term order with
  per-term scales**, so a policy trained with a given observation layout is
  reproduced exactly on every backend.
- [`PDController`](src/sim2sim/control/actuation.py) maps actions to torques
  with one shared law: `tau = Kp·(default + scale·action − q) − Kd·qd`.

Adding a new simulator is one file implementing the
[`Simulator`](src/sim2sim/sim/base.py) interface.

---

## Hardware / availability matrix

| Simulator | Install | Hardware | Runs in CI / here |
|-----------|---------|----------|-------------------|
| **MuJoCo** | `pip install mujoco` | CPU | ✅ yes (reference backend) |
| **PyBullet** | `pip install pybullet` | CPU | ✅ yes |
| **Genesis** | `pip install genesis-world` | **NVIDIA GPU + CUDA** | ⚠️ GPU host only |
| **Isaac Lab** | NVIDIA Isaac Sim stack (out-of-band) | **NVIDIA GPU + CUDA** | ⚠️ GPU host only |

The Genesis and Isaac Lab adapters are written against their real APIs but
cannot execute on a CPU host. They are **import-guarded**: the registry reports
them unavailable and skips them instead of crashing. Their live behavior is
validated on a GPU host (see [GPU runbook](#gpu-runbook)).

---

## Install

```bash
# CPU backends (MuJoCo + PyBullet) + dev tools
pip install -e ".[all-cpu,report]"

# or individually
pip install -e ".[mujoco]"
pip install -e ".[pybullet]"
pip install -e ".[genesis]"     # GPU host
# Isaac Lab is installed via the NVIDIA Isaac Sim installer, not this extra.

# everything for development (tests, lint, both CPU sims, onnx)
pip install -e ".[dev]"
```

## Quickstart

```bash
# Which backends are available on this machine?
sim2sim list-sims

# Evaluate across all configured simulators (random baseline if no ONNX set)
sim2sim eval --config configs/eval.yaml --out report

# Restrict to specific simulators / policy kind
sim2sim eval --config configs/eval.yaml --sims mujoco,pybullet --policy random
```

Output is a markdown table + bar-chart PNG in `report/`:

```
| Metric              | mujoco          | pybullet        |
|---------------------|-----------------|-----------------|
| Survival rate ↑     | 1.000 ± 0.000   | 0.000 ± 0.000   |
| Survival time (s) ↑ | 10.000 ± 0.000  | 0.220 ± 0.000   |
| Distance (m) ↑      | 0.163 ± 0.000   | 0.005 ± 0.000   |
| Lin-vel err ↓       | 0.620 ± 0.216   | 0.617 ± 0.218   |
| ...                 | ...             | ...             |
```

(The numbers above are the **random baseline** on the bundled toy robot — they
illustrate the report, not a good policy. Drop in a trained ONNX policy for a
meaningful comparison.)

## Using your own policy

1. Export your locomotion policy to ONNX (single input → single output).
2. Point `configs/policy/quad12_flat.yaml` at it and make `obs_terms` match the
   **exact observation layout and scales** the policy was trained with.
3. Run `sim2sim eval`. The obs dimension is validated against the ONNX graph at
   load time, so a layout mismatch fails fast instead of producing garbage.

## The bundled robot (`quad12`)

A simplified 12-DOF quadruped (4 legs × {hip, thigh, calf}) whose joint topology
mirrors a Unitree Go2. It ships as both [MJCF](src/sim2sim/assets/quad12/quad12.xml)
(MuJoCo/Genesis) and [URDF](src/sim2sim/assets/quad12/quad12.urdf) (PyBullet) so
the **same morphology** is loaded everywhere. It is a generic stand-in — for a
high-fidelity study, drop a real Go2 MJCF/URDF (e.g. from `mujoco_menagerie`)
into `assets/` and point `configs/robot/quad12.yaml` at it. No code change is
needed: the robot is entirely config-driven.

## Configuration

| File | Purpose |
|------|---------|
| `configs/robot/quad12.yaml` | joint order, default pose, PD gains, action scale, asset paths |
| `configs/policy/quad12_flat.yaml` | observation term order + scales, command ranges, ONNX path |
| `configs/eval.yaml` | which sims, episodes, seeds, control rate, fall thresholds |

## Metrics

Per-episode, aggregated to mean ± std across seeds:
survival rate / time, distance travelled, linear & angular **velocity-tracking
error**, mean torque, **cost of transport**, and action smoothness.

## GPU runbook

On a host with an NVIDIA GPU + CUDA:

```bash
pip install -e ".[genesis,report]"          # Genesis
sim2sim eval --config configs/eval.yaml --sims mujoco,pybullet,genesis

# Isaac Lab: install Isaac Sim + Isaac Lab per NVIDIA docs, then run with the
# Isaac Lab python:
./isaaclab.sh -p -m sim2sim.cli eval --config configs/eval.yaml --sims isaaclab
```

GPU-only tests are marked `@pytest.mark.gpu` (none required to ship; the CPU
suite fully exercises the shared pipeline).

## Testing

```bash
pytest -m "not gpu"        # full CPU suite (parity, actuation, ONNX, smoke)
ruff check src tests
ruff format --check src tests
```

The CPU suite covers the sim-to-sim contract directly: observation parity,
deterministic actuation, the ONNX obs-dim contract, registry availability, and
end-to-end MuJoCo + PyBullet rollouts.

## Layout

```
src/sim2sim/
  config.py            # dataclasses + YAML loader
  sim/                 # Simulator ABC, RobotState, registry, 4 adapters
  obs/                 # ObservationBuilder + command generator
  control/             # shared action->torque PD law
  policy/              # Policy protocol, ONNX policy, baselines
  eval/                # metrics, runner, report
  cli.py               # `sim2sim eval` / `sim2sim list-sims`
  assets/quad12/       # bundled MJCF + URDF
configs/  examples/  tests/
```

## License

MIT.
