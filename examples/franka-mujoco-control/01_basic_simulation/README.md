# 01. Basic Simulation

This example demonstrates the simplest possible setup: loading and running a Franka Panda robot simulation using the `dora-mujoco` node.

- Understand how the `dora-mujoco` node works
- See how robot models are loaded from `robot-descriptions`
- Learn the basic dataflow for physics simulation

## Architecture

```
[Timer] → [MuJoCo Sim] → [Joint Positions, Velocities, Sensor Data]
```

The simulation runs at 500Hz and outputs:
- Joint positions for all robot joints
- Joint velocities 
- Sensor data (if available)
- Current simulation time

## Running the Example

```bash
cd 01_basic_simulation
dora build basic.yml
dora run basic.yml
```

You should see:
1. MuJoCo viewer window opens with Franka Panda robot
2. Robot is effected by gravity (enabled by default)
3. Console output showing node activity

## What's Happening

1. **Model Loading**: The `dora-mujoco` node loads the Franka model using `load_robot_description("panda_mj_description")`
2. **Physics Loop**: Timer triggers simulation steps at 500Hz (This is dafault step time for Mujoco)
3. **Data Output**: Joint states are published 
4. **Visualization**: MuJoCo viewer shows real-time simulation

## Configuration Details

The `basic.yml` configures:
- Model name: `"panda"` (resolved to `panda_mj_description`)
- Update rate: 2ms (500Hz)
- Outputs: Joint positions, velocities, and sensor data