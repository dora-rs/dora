# 01. Basic Simulation

This example demonstrates the simplest possible setup: loading and running a robot simulation using the `dora-mujoco` node.

- Understand how the `dora-mujoco` node works
- See how robot models are loaded from `robot-descriptions`
- Learn the basic dataflow for physics simulation


The simulation runs at 500Hz and outputs:
- Joint positions for all robot joints
- Joint velocities 
- Sensor data (if available)
- Current simulation time

### Running the Example

```bash
cd basic_simulation
dora build basic.yml
dora run basic.yml
```

You should see:
1. MuJoCo viewer window opens with GO2 robot
2. Robot is effected by gravity (enabled by default)

### What's Happening

1. **Model Loading**: The `dora-mujoco` node loads the RoboDog (go2) model using `load_robot_description("go2_mj_description")`
2. **Physics Loop**: Timer triggers simulation steps at 500Hz (This is default step time for Mujoco)
3. **Data Output**: Joint states are published 
4. **Visualization**: MuJoCo viewer shows real-time simulation

### Configuration Details

The `basic.yml` configures:
- Model name: `"go2"` you change this to other robots name
- Update rate: 2ms (500Hz) 
- Outputs: Joint positions, velocities, and sensor data