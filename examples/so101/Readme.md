# SO-101 Robotic Arm Control with Dora-rs

This project provides gamepad control and leader-follower functionality for the SO-101 robotic arm using the Dora-rs framework. It includes real-time inverse kinematics, gripper control, and 3D visualization through Rerun.

## Features

- **Gamepad Control**: Real-time control of the SO-101 arm using a gamepad controller
- **Inverse Kinematics**: Differential IK control for smooth end-effector positioning
- **Gripper Control**: Precise gripper operation with button controls
- **3D Visualization**: Real-time robot visualization using Rerun
- **Leader-Follower Mode**: Map one arm's pose to control another arm
- **Speed Scaling**: Adjustable movement speed with gamepad bumpers

## Prerequisites

- Python 3.8+
- Dora-rs framework
- Compatible gamepad controller
- SO-101 robotic arm(s) with serial connection

## Installation

### 1. Install Dependencies

install the required Python packages for rerun visualization (optional):

```bash
# Install the URDF loader for Rerun visualization
pip install git+https://github.com/dora-rs/rerun-loader-python-urdf
```
### 2. Download URDF Files

The URDF files and assets for So-arm 101 can be downloaded from here:

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
- **Gamepad input** for real-time control using [gamepad node](https://github.com/dora-rs/dora/tree/add-robot-descriptions/node-hub/gamepad)
- **Inverse kinematics controller** for smooth movement
- **Physical arm interface** via serial communication via `dora-rustypot` node
- **3D visualization** in Rerun

### Leader-Follower Mode (leader.yml)

Use one arm as a leader to control another follower arm:

```bash
dora build leader.yml
dora run leader.yml
```

This dataflow includes:
- **Leader arm** (manual control)
- **Follower arm** (mirrors leader movements)
- **3D visualization** of the follower arm

## Gamepad Controls

### Movement Controls
> NOTE: the mapping may differ based your controller.
- **D-pad**: Move end-effector in X-Y plane
- **Right Stick**: Move end-effector in Z axis and rotate around Z (yaw)

### Gripper Controls
- **Button A**: Open gripper (increase angle)
- **Button B**: Close gripper (decrease angle)
- Gripper range: 0° (closed) to 95° (fully open)

### Speed Controls
- **Left Bumper (LB)**: Decrease movement speed
- **Right Bumper (RB)**: Increase movement speed
- Speed range: 0.1x to 1.0x

## Configuration

### Serial Port Configuration

Update the `PORT` environment variable in the YAML files:

```yaml
env:
  PORT: /dev/ttyACM0  # Change to your actual port
  IDS: 1 2 3 4 5 6    # Servo IDs
```

### URDF Path Configuration

Ensure the URDF path is correct in the configuration:

```yaml
env:
  URDF_PATH: SO101/so101_new_calib.urdf # make sure this path is correct
  so101_new_calib_urdf: SO101/so101_new_calib.urdf
```

## Control Parameters

The controller includes several tunable parameters in `so_101_controller.py`:

- `Kpos`: Position gain (default: 0.95)
- `Kori`: Orientation gain (default: 0.95)
- `damping`: IK damping factor (default: 1e-4)
- `max_angvel`: Maximum joint velocity (default: 1.57 rad/s)
- `gripper_rate`: Gripper movement rate (default: 5.0 deg/s)

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
- Keep the emergency stop (if available) within reach
