# dora-mujoco

A MuJoCo physics simulation node for the Dora dataflow framework. This node provides real-time physics simulation with support for a wide range of robots from the `robot_descriptions` package, as well as custom robot models. Designed for modular robotics control architectures.

## Features

- **Wide Robot Support**: Built-in support for 50+ robot models including quadrupeds, humanoids, arms, drones, and more
- **Flexible Model Loading**: Load robots by name (via robot_descriptions) or direct XML file paths
- **Real-time Simulation**: Continuous physics simulation with configurable tick rates
- **Live Visualization**: Optional MuJoCo viewer for real-time 3D visualization
- **Generic Control Interface**: Accepts control commands for any robot type
- **Rich Data Output**: Joint positions, velocities, accelerations, and sensor data


## Supported Robot Categories

| Category | Examples |
|----------|----------|
| **Quadrupeds** | Unitree Go1/Go2/A1, ANYmal B/C, Boston Dynamics Spot |
| **Humanoids** | Unitree G1/H1, Apptronik Apollo, TALOS, JVRC-1 |
| **Arms** | Franka Panda, KUKA iiwa14, Universal Robots UR5e/UR10e |
| **Dual Arms** | Aloha 2, Baxter, YuMi |
| **End Effectors** | Allegro Hand, Shadow Hand, Robotiq 2F-85 |
| **Drones** | Crazyflie 2.0, Skydio X2 |
| **Educational** | Double Pendulum |

## Getting Started

### Quick Start

1. **Run a simple simulation**:
```bash
dora build demo.yml 
# Start with Unitree Go2 quadruped
dora run demo.yml
```

2. **Use different robots**:
```python
# In main.py, modify the model name (line ~95)
model_path_or_name = "franka_panda"  # Change this line
# Examples: "go2", "franka_panda", "g1", "spot", etc.
```

3. **Use custom robot models**:
```python
# In main.py, use file path instead of model name
model_path_or_name = "/path/to/my_robot/scene.xml"
```

## Usage Examples
TODO

### Available Robot Models
The simulator supports all models from the `robot_descriptions` package. Common names include:

- **Quadrupeds**: `go1`, `go2`, `a1`, `aliengo`, `anymal_b`, `anymal_c`, `spot`
- **Humanoids**: `g1`, `h1`, `apollo`, `talos`, `jvrc`, `cassie`
- **Arms**: `panda`, `ur5e`, `ur10e`, `iiwa14`, `gen3`, `sawyer`
- **Hands**: `allegro_hand`, `shadow_hand`, `leap_hand`, `robotiq_2f85`

See [`robot_models.json`](dora_mujoco/robot_models.json) for the complete list.

## Architecture

The `dora-mujoco` node is designed to be **robot-agnostic** and work with **robot-specific controllers**:

```
┌─────────────────┐    
│  Command Node   │    target_pose/cmd_vel
│  (Gamepad, etc) │─────────────┐
└─────────────────┘             │
                                ▼
┌─────────────────┐    control_input   ┌─────────────────┐
│ Robot Controller│───────────────────▶│  dora-mujoco    │
│ (Franka, Husky) │                    │                 │
└─────────────────┘                    │ ┌─────────────┐ │    joint_positions
           ▲                           │ │  Simulator  │ │──────────────────▶
           │        joint_positions    │ │             │ │    joint_velocities  
           └───────────────────────────│ │  ┌────────┐ │ │──────────────────▶
                                       │ │  │Renderer│ │ │    actuator_controls
                                       │ │  └────────┘ │ │──────────────────▶
                                       │ │             │ │    sensor_data
                                       │ └─────────────┘ │──────────────────▶
                                       └─────────────────┘
the control nodes are optional (the robot will still spawn without them)
```

## YAML Specification

### Node Configuration
```yaml
nodes:
  - id: mujoco_sim
    build: pip install -e .
    path: dora-mujoco
    env:
      MODEL_NAME: "go2"  # Robot model name or file path
    inputs:
      TICK: dora/timer/millis/2  # Simulation tick rate (500Hz)
    outputs:
      - joint_positions    # Joint position array
      - joint_velocities   # Joint velocity array  
      - sensor_data       # Sensor readings array
```

### Input Specification
| Input | Type | Description |
|-------|------|-------------|
| `tick` | Timer | Triggers simulation steps and data output |
| `control_input` | `pa.array(float64)` | Control commands for actuators |

**Control Input Format**:
- **Manipulator Arms**: Joint position commands `[q1, q2, ..., q7, gripper]`
- **Mobile Robots**: Wheel velocity commands `[wheel1, wheel2, wheel3, wheel4]`
- **Quadrupeds**: Joint position commands `[hip1, thigh1, calf1, ...]`

### Output Specification
| Output | Type | Description | Metadata |
|--------|------|-------------|----------|
| `joint_positions` | `pa.array(float64)` | Joint positions (qpos) | `timestamp` |
| `joint_velocities` | `pa.array(float64)` | Joint velocities (qvel) | `timestamp` |
| `actuator_controls` | `pa.array(float64)` | Current actuator commands | `timestamp` |
| `sensor_data` | `pa.array(float64)` | Sensor readings (if available) | `timestamp` |

## Configuration Options

### Environment Variables
- `MODEL_NAME`: Robot model name or XML file path (default: "go2")

### Custom Robot Models
```python
# Use direct XML file path
env:
  MODEL_NAME: "/path/to/my_robot/scene.xml"
```

### Simulation Parameters
- **Physics Timestep**: Fixed at MuJoCo default (0.002s = 500Hz)
- **Output Rate**: Controlled by `tick` input frequency
- **Control Rate**: Determined by `control_input` frequency

## Development

### Code Quality
```bash
# Format code
uv run ruff check . --fix

# Lint code  
uv run ruff check .

# Run tests
uv pip install pytest
uv run pytest .
```

### Adding New Models
To add support for new robot models:

1. Add the model mapping to [`robot_models.json`](dora_mujoco/robot_models.json):
```json
{
  "category_name": {
    "my_robot": "my_robot_mj_description"
  }
}
```

2. Ensure the model is available in `robot_descriptions` or provide the XML file path directly.

### Creating Robot Controllers

Robot-specific controllers should:
1. Subscribe to `joint_positions` from the simulation
2. Implement robot-specific control logic (IK, dynamics, etc.)
3. Publish `control_input` commands to the simulation



## Troubleshooting

### Common Issues

1. **Model not found**:
   ```
   ERROR: Model file not found for my_robot
   ```
   - Check if the model name exists in `robot_models.json`
   - Verify `robot_descriptions` has the model installed
   - Use absolute file path for custom models

2. **Scene variant missing**:
   ```
   WARNING: Failed to load scene variant
   ```
   - Normal behavior - falls back to robot-only model
   - Scene variants include ground plane and lighting

3. **Control dimension mismatch**:
   ```
   WARNING: Control input size doesn't match actuators
   ```
   - Check that control commands match the robot's actuator count
   - Use `print(model.nu)` to see expected control dimensions

4. **Viewer issues**:
   - Ensure proper OpenGL/graphics drivers
   - Use headless mode for server environments

## Examples

Complete working examples are available in:
- `dora/examples/franka-mujoco-control/`
  - Target pose control with Cartesian space control
  - Gamepad control with real-time interaction

## License

This project is released under the MIT License. See the LICENSE file for details.

## Contributing

Contributions are welcome! Please:

1. Follow the existing code style (ruff formatting)
2. Add tests for new features
3. Update documentation as needed
4. Submit pull requests with clear descriptions

## Related Projects

- [Dora-rs](https://github.com/dora-rs/dora) - The dataflow framework
- [MuJoCo](https://github.com/google-deepmind/mujoco) - Physics simulation engine  
- [robot_descriptions](https://github.com/robot-descriptions/robot_descriptions) - Robot model collection
