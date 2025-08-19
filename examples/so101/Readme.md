## SO101 Arm Control

This example provides gamepad control and leader-follower functionality for the SO-101 robotic arm.

### Install Dependencies

install the required Python packages for rerun visualization (optional):

```bash
# Install the URDF loader for Rerun visualization
pip install git+https://github.com/dora-rs/rerun-loader-python-urdf
```

### Hardware Setup

1. Connect your SO-101 arm(s) to your computer via USB/serial
2. Note the serial port names (e.g.,for linux `/dev/ttyACM0`, `/dev/ttyACM1`)
3. Connect your gamepad controller
4. Update the `PORT` environment variable in the YAML files

#### Single Arm Control (arm_gamepad_control.yml)

Control a single SO-101 arm with gamepad input and visualization:

```bash
dora build arm.yml
dora run arm.yml
```

#### Leader-Follower Mode (leader_follower.yml)

Use one arm as a leader to control another follower arm:

```bash
dora build leader.yml
dora run leader.yml
```

#### Serial Port Configuration

Update the `PORT` environment variable in the YAML files:

```yaml
env:
  PORT: /dev/ttyACM0  # Change to your actual port
```

## Troubleshooting

### Serial Connection Issues
- Check that the arm is powered on and connected
- Verify the correct serial port in the YAML configuration
- Ensure proper permissions: `sudo chmod +x PORT`

### Gamepad Not Detected
- Verify gamepad is connected and recognized by the system
- Test with `jstest /dev/input/js0` (Linux)

## Safety Notes
- Always ensure the arm has sufficient clearance before operation