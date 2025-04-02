# dora-mujoco-husky

A MuJoCo-based Clearpath Husky simulation node for the Dora framework. This node provides a physics-accurate simulation of the Husky robot that can be controlled via velocity commands.

## Features

- Real-time physics simulation using MuJoCo
- Accurate Husky robot model with proper inertial properties
- Velocity-based control interface
- Position and velocity feedback
- Gamepad control support

## Getting Started

### Prerequisites

- Python 3.8 or higher
- MuJoCo 3.1.6 or higher

### Installation

1. Create and activate a Python virtual environment:
```bash
uv venv -p 3.11 --seed
source .venv/bin/activate
```

2. Install the package:
```bash 
uv pip install -e .
```

### Running the Demo

1. Make sure you have a gamepad connected
2. Build the demo:
```bash
dora build demo.yml --uv
```
3. Run the demo:
```bash
dora run demo.yml --uv
```

## Usage

The node accepts velocity commands and outputs position/velocity data:

### Inputs

- `cmd_vel`: Velocity command array with the format:
  - Index 0: Linear velocity (m/s), range: [-2.0, 2.0]
  - Index 5: Angular velocity (rad/s), range: [-3.0, 3.0]

### Outputs

- `position`: Robot position [x, y, z] in world coordinates
- `velocity`: Robot velocity [vx, vy, vz] in world coordinates

### Example Configuration

```yaml
nodes:
  - id: mujoco_husky
    build: pip install -e .
    path: dora-mujoco-husky
    inputs:
      cmd_vel: input_source/cmd_vel
    outputs:
      - position
      - velocity
```

## Development

### Code Formatting

Format code using ruff:
```bash
uv pip install ruff
uv run ruff check . --fix
```

### Linting

Run linting checks:
```bash
uv run ruff check .
```

### Testing

Run tests using pytest:
```bash
uv pip install pytest
uv run pytest .
```

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- Based on the Clearpath Robotics Husky robot
- Uses the MuJoCo physics engine
- Built with the Dora framework