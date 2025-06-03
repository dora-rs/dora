# dora-mujoco-sim

A MuJoCo physics simulation node for the Dora dataflow framework. This node provides real-time physics simulation with support for a wide range of robots from the `robot_descriptions` package, as well as custom robot models.

## Features

- **Wide Robot Support**: Built-in support for 50+ robot models including quadrupeds, humanoids, arms, drones, and more
- **Flexible Model Loading**: Load robots by name (via robot_descriptions) or direct XML file paths
- **Real-time Simulation**: Continuous physics simulation with configurable tick rates
- **Live Visualization**: Optional MuJoCo viewer for real-time 3D visualization
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
- **Arms**: `franka_panda`, `ur5e`, `ur10e`, `iiwa14`, `gen3`, `sawyer`
- **Hands**: `allegro_hand`, `shadow_hand`, `leap_hand`, `robotiq_2f85`

See [`robot_models.json`](dora_mujoco_sim/robot_models.json) for the complete list.

## YAML Specification

### Node Configuration
```yaml
nodes:
  - id: mujoco_sim
    build: pip install -e .
    path: dora-mujoco-sim
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
- **TICK**: Timer events that trigger simulation steps and data output
  - Format: `dora/timer/millis/<interval>`
  - Example: `dora/timer/millis/1` for 1000Hz updates

### Output Specification
| Output | Type | Description | Metadata |
|--------|------|-------------|----------|
| `joint_positions` | `pa.array(float64)` | Joint positions (qpos) | `timestamp` |
| `joint_velocities` | `pa.array(float64)` | Joint velocities (qvel) | `timestamp` |
| `sensor_data` | `pa.array(float64)` | Sensor readings (yet to manage sensors, currently dumping raw output) | `timestamp` |


## Architecture

```
┌─────────────────┐    TICK    ┌─────────────────┐
│  Dora Timer     │───────────▶│  MuJoCo Node    │
└─────────────────┘            │                 │
                               │ ┌─────────────┐ │    joint_positions
                               │ │  Simulator  │ │──────────────────▶
                               │ │             │ │    joint_velocities  
                               │ │  ┌────────┐ │ │──────────────────▶
                               │ │  │Renderer│ │ │    sensor_data
                               │ │  └────────┘ │ │──────────────────▶
                               │ │             │ │
                               │ └─────────────┘ |
                               └─────────────────┘
```

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

1. Add the model mapping to [`robot_models.json`](dora_mujoco_sim/robot_models.json):
```json
{
  "category_name": {
    "my_robot": "my_robot_mj_description"
  }
}
```

2. Ensure the model is available in `robot_descriptions` or provide the XML file path directly.

## Configuration


### Simulation Parameters
- **Timestep**: Fixed at MuJoCo default (0.002s = 500Hz physics)
- **Output Rate**: Controlled by TICK input frequency

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

3. **Viewer issues**:
   - Use `HEADLESS` for server environments
   - Ensure proper OpenGL/graphics drivers

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
