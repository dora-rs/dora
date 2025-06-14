# 03. Gamepad Control

This example demonstrates real-time interactive control by connecting a gamepad to the controller. It builds upon the target pose control example by adding gamepad input processing. Shows how to integrate and implement teleoperation. 

### Gamepad Controls:
- **D-pad Vertical**: Move along X-axis (forward/backward) 
- **D-pad Horizontal**: Move along Y-axis (left/right)
- **Right Stick Y**: Move along Z-axis (up/down)
- **LB/RB**: Decrease/Increase movement speed (0.1-1.0x scale)
- **START**: Reset to home position

## Running the Example

1. **Connect a gamepad** (Xbox/PlayStation controller via USB or Bluetooth)
2. **Run the simulation**:
```bash
cd gamepad_control
dora build gamepad_control.yml
dora run gamepad_control.yml
```

You should see:
1. Robot responds to gamepad input in real-time
2. Smooth incremental movement based on D-pad/stick input
3. Speed control with bumper buttons
4. Reset functionality with START button
5. GPU-accelerated kinematics computation (if CUDA available)

#### **Gamepad Node** (`gamepad`)
Built-in Dora node that interfaces with system gamepad drivers.
- **Outputs**: 
  - `cmd_vel`: 6DOF velocity commands `[linear_x, linear_y, linear_z, angular_x, angular_y, angular_z]`
  - `raw_control`: Gamepad Input in Json format 

#### **GamePad Controller** (`gamepad_controller.py`)
This node processes gamepad input and translates it into target positions for the robot end-effector.

- Receives continuous movement commands (`cmd_vel`) from D-pad and analog sticks
- Processes discrete button presses (`raw_control`) for special functions

**Controlling the End-Effector with Gamepad**:

The controller takes the first 3 values (X, Y, Z movement) from the gamepad `cmd_vel`, updates Target Position continuously

**Button Commands**:
    - **START button**: resets end-effector to home position [0.4, 0.0, 0.3]
    - **Left Bumper (LB)**: Decreases movement speed 
    - **Right Bumper (RB)**: Increases movement speed 

The end-effector moves smoothly as you hold the controls, with position updates sent to the inverse kinematics solver to calculate required joint angles.

## Troubleshooting

**"Gamepad not responding"**
```bash
# Check if gamepad is connected
ls /dev/input/js*
# Test gamepad input
jstest /dev/input/js0
```

**"Robot doesn't move with D-pad"**
- Check `cmd_vel` output: should show non-zero values when D-pad pressed
- Verify controller processes `cmd_vel` events
- Ensure your gamepad has correct mapping.


## Real Robot Deployment

For actual robot control:
1. **Replace MuJoCo**: Use real robot drivers
2. **Safety Limits**: Add emergency stops and workspace bounds
3. **Force Control**: Integrate force/torque feedback
4. **Network Latency**: Consider wireless controller delay
5. **Deadman Switch**: Require constant button hold for safety
