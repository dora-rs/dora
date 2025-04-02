# Gamepad Node

A Dora framework node that provides universal gamepad control functionality. While tested primarily with the Logitech F710 wireless controller, it supports any standard gamepad/joystick.

## Features

- Universal gamepad support (USB/Wireless)
- Auto-detection of connected controllers
- Configurable axis mapping for different controllers
- Dead zone handling for precise control
- Real-time velocity command output
- Clean shutdown with automatic zero velocity
- Customizable speed limits
- Default configuration optimized for Logitech F710

## Getting Started

### Prerequisites

- Python 3.8 or higher
- Dora Framework 0.3.6 or higher
- Any compatible USB/Wireless gamepad

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

- **Left Stick Y-Axis**: Forward/Backward movement (±2.0 m/s)
- **Left Stick X-Axis**: Left/Right turning (±1.5 rad/s)
- **Deadzone**: 5% to prevent drift

### YAML Specification

```yaml
nodes:
  - id: gamepad
    build: pip install -e .
    path: gamepad
    outputs:
      - cmd_vel  # Velocity commands [vx, vy, vz, wx, wy, wz]
    inputs:
      tick: dora/timer/millis/10  # Update rate
```

### Outputs

The node outputs `cmd_vel` as a 6-element array:
- `[0]`: Linear X velocity (forward/backward)
- `[1]`: Linear Y velocity (always 0)
- `[2]`: Linear Z velocity (always 0)
- `[3]`: Angular X velocity (always 0)
- `[4]`: Angular Y velocity (always 0)
- `[5]`: Angular Z velocity (turning) 

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

## Troubleshooting

**No gamepad detected**: 
 - Check USB receiver connection
 - Ensure controller is powered on
 - Verify mode switch position

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