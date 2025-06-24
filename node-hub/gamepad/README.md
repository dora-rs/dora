# Gamepad Node

A Dora framework node that provides universal gamepad control functionality. While tested primarily with the Logitech F710 wireless controller, it supports any standard gamepad/joystick.

## Features

- Universal gamepad support (USB/Wireless)
- Auto-detection of connected controllers
- Configurable axis mapping for different controllers
- Dead zone handling for precise control
- Real-time velocity command output
- Raw gamepad state output for custom processing
- Clean shutdown with automatic zero velocity
- Customizable speed limits
- Default configuration optimized for Logitech F710

## Getting Started

### Prerequisites

- Python 3.8 or higher
- Dora Framework 0.3.6 or higher
- Any compatible USB/Wireless gamepad
- Pygame library for joystick handling

### Installation

1. Install the package:
```bash
pip install -e .
```

2. Ensure your Logitech F710 is:
   - Connected via USB receiver
   - Powered on

### Running the Node

1. Basic usage:
```bash
dora build demo.yml --uv

dora run demo.yml --uv
```

2. Check the controller is detected:
```bash
ls /dev/input/js*  # Should show your controller
```
If port is not executable then run
```bash
sudo chmod +x /dev/input/js*
```

## Usage

### Controls

#### Default Control Mapping (cmd_vel output):
- **D-pad Vertical**: Linear X movement (forward/backward)
- **D-pad Horizontal**: Linear Y movement (left/right)
- **Right Stick Y-Axis**: Linear Z movement (up/down, ±0.1 m/s)
- **Right Stick X-Axis**: Angular Z rotation (turning, ±0.8 rad/s)
- **Left Stick Y-Axis**: Angular X rotation (±0.8 rad/s)
- **Left Stick X-Axis**: Angular Y rotation (±0.8 rad/s)
- **Deadzone**: 5% to prevent drift

#### Raw Control Output:
The node also outputs complete gamepad state including all buttons, axes, and D-pad for custom processing.

### YAML Specification

```yaml
nodes:
  - id: gamepad
    build: pip install -e .
    path: gamepad
    outputs:
      - cmd_vel      # Velocity commands [vx, vy, vz, wx, wy, wz]
      - raw_control  # Complete gamepad state (JSON)
    inputs:
      tick: dora/timer/millis/10  # Update rate (100Hz)
```

### Outputs

#### 1. `cmd_vel` - 6-element velocity array:
- `[0]`: Linear X velocity (D-pad vertical)
- `[1]`: Linear Y velocity (D-pad horizontal)
- `[2]`: Linear Z velocity (right stick Y)
- `[3]`: Angular X velocity (left stick Y)
- `[4]`: Angular Y velocity (left stick X)
- `[5]`: Angular Z velocity (right stick X)

#### 2. `raw_control` - Complete gamepad state (JSON):
```json
{
  "axes": [left_x, left_y, right_x, right_y, ...],
  "buttons": [X, A, B, Y, LB, RB, LT, RT, BACK, START, LEFT_STICK, RIGHT_STICK],
  "hats": [[dpad_x, dpad_y]],
  "mapping": {
    "axes": {"LEFT-X": 0, "LEFT-Y": 1, "RIGHT-X": 2, "RIGHT-Y": 3},
    "buttons": {"X": 0, "A": 1, "B": 2, "Y": 3, "LB": 4, "RB": 5, ...}
  }
}
```

## Controller Configuration

To use a different controller, modify the `Controller` class mapping:

```python
class Controller:
    def __init__(self):
        # Customize for your controller
        self.axisNames = {
            'LEFT-X': 0,
            'LEFT-Y': 1,        
            'RIGHT-X': 2,
            'RIGHT-Y': 3,   
        }
        self.buttonNames = {
            'X': 0,
            'A': 1,
            # ... add your button mapping
        }
```

## Development

### Code Style

Format code using ruff:
```bash
ruff check . --fix
```

### Testing

Run tests with pytest:
```bash
pytest .
```

## Integration Examples

### Simple Velocity Control:
```python
for event in node:
    if event["id"] == "cmd_vel":
        velocity = event["value"].to_numpy()
        # Apply velocity to your robot/system
        print(f"Velocity: {velocity}")
```

### Custom Button Processing:
```python
import json

for event in node:
    if event["id"] == "raw_control":
        control_state = json.loads(event["value"].to_pylist()[0])
        buttons = control_state["buttons"]
        axis = control_state[axis]
```

## Troubleshooting

**No gamepad detected**: 
 - Check USB receiver connection
 - Ensure controller is powered on
 - Verify mode switch position


**Raw control output empty**:
 - Verify gamepad is responding with `jstest /dev/input/js0`
 - Check pygame initialization

**Incorrect button mapping**:
 - Different controllers have different mappings
 - Use `jstest` to identify your controller's button/axis numbers
 - Update the `Controller` class accordingly

**If port is not executable then run:**
```bash
sudo chmod +x /dev/input/js*
```

## License

Released under the MIT License. See LICENSE file for details.

## Acknowledgments

- Uses Pygame for joystick handling
- Built with the Dora framework
- Designed for Logitech F710 gamepad
- Supports any standard USB/Bluetooth gamepad