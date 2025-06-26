## SO-101 Robotic Arm Control

This example provides gamepad control and leader-follower functionality for the SO-101 robotic arm.

### 1. Install Dependencies

install the required Python packages for rerun visualization (optional):

```bash
# Install the URDF loader for Rerun visualization
pip install git+https://github.com/dora-rs/rerun-loader-python-urdf
```
### 2. Download URDF Files

The URDF files and assets for So-arm 101 can be downloaded from:

```bash
# Use this cmd in the terminal to download the assets 
git clone --depth 1 https://huggingface.co/datasets/SGPatil/so_arm_101 && mv so_arm_101/SO101 . && rm -rf so_arm_101
```


### 3. Hardware Setup

1. Connect your SO-101 arm(s) to your computer via USB/serial
2. Note the serial port names (e.g.,for linux `/dev/ttyACM0`, `/dev/ttyACM1`)
3. Connect your gamepad controller
4. Update the `PORT` environment variables in the YAML files

## Usage

### Single Arm Control (arm.yml)

Control a single SO-101 arm with gamepad input and visualization:

```bash
dora build arm.yml
dora run arm.yml
```

This dataflow includes:
- **Gamepad input** for real-time control using `gamepad node`
- **Pytorch kinematics** for inverse kinematics.
- **Physical arm interface** via serial communication via `dora-rustypot` node

### Leader-Follower Mode (leader.yml)

Use one arm as a leader to control another follower arm:

```bash
dora build leader.yml
dora run leader.yml
```

This dataflow includes:
- **Leader arm** (manual control)
- **Follower arm** (mirrors leader movements)

#### Serial Port Configuration

Update the `PORT` environment variable in the YAML files:

```yaml
env:
  PORT: /dev/ttyACM0  # Change to your actual port
  IDS: 1 2 3 4 5 6    # Servo IDs
```

#### URDF Path Configuration

Ensure the URDF path is correct in the configuration:

```yaml
env:
  URDF_PATH: SO101/so101_new_calib.urdf # make sure this path is correct
  so101_new_calib_urdf: SO101/so101_new_calib.urdf
```

## Troubleshooting

### Serial Connection Issues
- Check that the arm is powered on and connected
- Verify the correct serial port in the YAML configuration
- Ensure proper permissions: `sudo usermod -a -G dialout $USER`

### Gamepad Not Detected
- Verify gamepad is connected and recognized by the system
- Test with `jstest /dev/input/js0` (Linux)

### URDF Loading Errors
- Ensure all URDF files and mesh assets are in the correct location
- Verify the installation of `rerun-loader-python-urdf` plugin

## Safety Notes

- Always ensure the arm has sufficient clearance before operation

