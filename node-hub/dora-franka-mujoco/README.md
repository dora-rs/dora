# dora-franka-mujoco

A MuJoCo-based Franka Emika Panda robot simulation node for the Dora dataflow framework. This node provides high-fidelity physics simulation with real-time control capabilities, supporting both gamepad and programmatic control interfaces.

## Features

- **High-Fidelity Simulation**: Physics-based simulation using MuJoCo with detailed Franka Panda model
- **Dual Control Modes**: Support for both gamepad control and target pose commands
- **Cartesian Control**: End-effector position and orientation control with nullspace joint control
- **Real-time Feedback**: Joint positions and end-effector position streaming
- **Automatic Mesh Management**: Automatic download of required mesh files from Hugging Face
- **Interactive Visualization**: Built-in MuJoCo viewer for real-time monitoring

## Robot Model

- **Robot**: Franka Emika Panda (7-DOF arm + 2-DOF gripper)
- **Environment**: Table workspace with movable objects
- **Physics**: Full dynamics simulation with gravity and collision detection
- **Control**: Position control with damped least squares inverse kinematics

## Getting Started

### Installation

```bash
# Create virtual environment
uv venv -p 3.11 --seed

# Install the package
uv pip install -e .
```

### Quick Start

1. **Run with gamepad control**:
```bash
dora build demo.yml
# run the node 
dora run demo.yml
```

2. **Connect a gamepad** (Xbox/PlayStation controller) and use the controls below

### Controls

#### Gamepad Mapping
- **Left Stick X**: Move along X-axis
- **Left Stick Y**: Move along Y-axis  
- **Right Stick Y**: Move along Z-axis
- **LB/RB**: Decrease/Increase movement speed
- **START**: Reset robot to home position
- **X Button**: Close gripper
- **Y Button**: Open gripper

#### Programmatic Control
Send target poses as `[x, y, z, roll, pitch, yaw]` arrays to the `target_pose` input.

## YAML Specification

### Node Configuration
```yaml
nodes:
  - id: mujoco_franka
    build: pip install -e .
    path: dora-franka-mujoco
    inputs:
      raw_control: gamepad/raw_control  # Gamepad input (optional)
      target_pose: controller/target_pose  # Target pose commands (optional)
      tick: dora/timer/millis/10  # Simulation tick rate
    outputs:
      - joint_positions  # 7-DOF joint angles
      - ee_position     # End-effector position [x, y, z]
```

### Input Specifications

| Input | Type | Description | Format |
|-------|------|-------------|---------|
| `raw_control` | `pa.string()` | Gamepad input (handled by gamepad node) | `{"axes": [float], "buttons": [bool]}` |
| `target_pose` | `pa.array(float64)` | Target pose command | `[x, y, z, roll, pitch, yaw]` (position in meters, orientation in degrees) |
| `tick` | Timer | Simulation step trigger | Timer event |

### Output Specifications

| Output | Type | Description | Metadata |
|--------|------|-------------|----------|
| `joint_positions` | `pa.array(float64)` | 7-DOF joint angles (radians) | `{"type": "joint_positions"}` |
| `ee_position` | `pa.array(float64)` | End-effector position [x, y, z] (meters) | `{"type": "ee_position"}` |

## Examples

### Basic Simulation
```yaml
# Minimal setup for physics simulation
nodes:
  - id: mujoco_franka
    build: pip install -e .
    path: dora-franka-mujoco
    inputs:
      tick: dora/timer/millis/10
    outputs:
      - joint_positions
      - ee_position
```

### Gamepad Control
```yaml
# Full gamepad control setup
nodes:
  - id: gamepad
    build: pip install -e ../gamepad
    path: gamepad
    outputs:
      - raw_control
    inputs:
      tick: dora/timer/millis/10

  - id: mujoco_franka
    build: pip install -e .
    path: dora-franka-mujoco
    inputs:
      raw_control: gamepad/raw_control
      tick: dora/timer/millis/10
    outputs:
      - joint_positions
      - ee_position
```

### Programmatic Control
```python
# Send target pose commands
import pyarrow as pa
from dora import Node

node = Node()

# Move to position (0.5, 0.2, 0.6) with downward orientation
target_pose = [0.5, 0.2, 0.6, 180.0, 0.0, 90.0]  # x, y, z, roll, pitch, yaw

node.send_output(
    "target_pose",
    pa.array(target_pose, type=pa.float64()),
    metadata={"timestamp": time.time()}
)
```

## Technical Details

### Control System
- **Inverse Kinematics**: Damped least squares with nullspace control
- **Target Orientation**: Default downward-facing gripper (configurable)
- **Joint Limits**: Enforced according to Franka specifications
- **Velocity Limits**: Maximum 0.785 rad/s per joint


### Mesh Files
The node automatically downloads required mesh files from Hugging Face Hub:
- **Repository**: `SGPatil/Mujoco_franka_meshes`
- **Cache Location**: `~/.cache/dora-mujoco-franka/meshes/`
- **File Types**: STL and OBJ mesh files for visual and collision geometry

### Simulation Parameters
- **Timestep**: 0.002s (500Hz physics simulation)
- **Integration**: 0.1s control integration time
- **Damping**: 1e-4 for numerical stability
- **Position Gains**: Kpos=0.95, Kori=0.95
- **Nullspace Gains**: [10, 10, 10, 10, 5, 5, 5]

## Troubleshooting

### Common Issues

1. **Mesh files not downloading**:
   ```
   Error downloading mesh files: [error details]
   ```
   - Check internet connection
   - Verify Hugging Face Hub access
   - Clear cache: `rm -rf ~/.cache/dora-mujoco-franka/`

2. **Gamepad not detected**:
   - Ensure gamepad is connected and recognized by OS
   - Test with `js_test` on Linux or similar tools
   - Check gamepad node configuration

3. **Robot not responding to commands**:
   - Verify input connections in YAML
   - Check that timer is triggering simulation steps
   - Use MuJoCo viewer to monitor robot state

4. **Simulation running slowly**:
   - Reduce timer frequency (increase interval)
   - Close other applications using GPU/CPU
   - Consider headless mode for better performance


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

### Adding New Features
1. Modify the `RobotController` class for new control modes
2. Update input/output specifications
3. Add corresponding tests
4. Update documentation

## Contributing

Contributions are welcome! Please:

1. Follow the existing code style (ruff formatting)
2. Add tests for new features
3. Update documentation as needed
4. Submit pull requests with clear descriptions

## License

This project is released under the MIT License. See the LICENSE file for details.

## Related Projects

- [Dora-rs](https://github.com/dora-rs/dora) - The dataflow framework
- [MuJoCo](https://github.com/google-deepmind/mujoco) - Physics simulation engine
- [Franka Control Interface](https://frankaemika.github.io/docs/) - Official Franka documentation
