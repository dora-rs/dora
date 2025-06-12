# 03. Gamepad Control

This example demonstrates real-time interactive control by connecting a gamepad to the Franka controller. It builds upon the target pose control example by adding gamepad input processing while using **PyTorch Kinematics** for efficient computation. Shows how to integrate multiple input sources and implement teleoperation with GPU-accelerated kinematics.

## Controls

- **Left Stick X**: Move along X-axis (forward/backward)
- **Left Stick Y**: Move along Y-axis (left/right) 
- **Right Stick Y**: Move along Z-axis (up/down)
- **LB/RB**: Decrease/Increase movement speed (0.1-1.0x scale)
- **START**: Reset to home position
- **X Button**: Close gripper
- **Y Button**: Open gripper

## Running the Example

1. **Connect a gamepad** (Xbox/PlayStation controller via USB or Bluetooth)
2. **Run the simulation**:
```bash
cd 03_gamepad_control
dora build gamepad_control.yml
dora run gamepad_control.yml
```

You should see:
1. Robot responds to gamepad input in real-time
2. Smooth incremental movement based on stick input
3. Speed control with bumper buttons
4. Gripper control with face buttons
5. GPU-accelerated kinematics computation (if CUDA available)

## How It Works

### Node Breakdown

#### 1. **Gamepad Node** (`gamepad`)
Built-in Dora node that interfaces with system gamepad drivers.

**What it does**:
- Polls connected gamepad at 100Hz (`tick: dora/timer/millis/10`)
- Converts raw gamepad input to standardized JSON format
- **Outputs**: 
  - `raw_control`: Raw gamepad data with axes and button states
  - `cmd_vel`: Velocity commands (unused in this example)

**Raw Control Format**:
```json
{
  "axes": [stick_x, stick_y, trigger_l, stick_rx, stick_ry, trigger_r],
  "buttons": [X, A, B, Y, LB, RB, ..., START, ...],
  "mapping": {"button_names": {...}, "axis_names": {...}}
}
```

#### 2. **Enhanced Franka Controller** (`franka_gamepad_controller.py`)

**Key Enhancement**: Extends the target pose controller with gamepad input processing using PyTorch Kinematics.

**Gamepad Input Processing**:
```python
def process_gamepad_input(self, raw_control):
    axes = raw_control["axes"]
    buttons = raw_control["buttons"]
    
    # 1. Button handling
    if buttons[9]:  # START button
        # Reset target to home position using PyTorch Kinematics FK
        home_ee_pos, _ = self.get_current_ee_pose(self.home_pos)
        self.target_pos = home_ee_pos.copy()
        print("Reset target to home position")
    
    # 2. Gripper control
    if buttons[0]:      # X button - Close gripper
        self.gripper_state = 0.0
        print("Gripper: CLOSED")
    elif buttons[3]:    # Y button - Open gripper  
        self.gripper_state = 1.0
        print("Gripper: OPEN")
    
    # 3. Speed scaling
    if buttons[4]: self.speed_scale = max(0.1, self.speed_scale - 0.1)  # LB
    if buttons[5]: self.speed_scale = min(1.0, self.speed_scale + 0.1)  # RB
    
    # 4. Incremental position control with deadzone
    dx = self.apply_deadzone(axes[0]) * self.speed_scale * 0.1
    dy = -self.apply_deadzone(axes[1]) * self.speed_scale * 0.1  # Inverted Y
    dz = -self.apply_deadzone(axes[3]) * self.speed_scale * 0.1  # Right stick Y
    
    # 5. Update target position incrementally
    if abs(dx) > 0 or abs(dy) > 0 or abs(dz) > 0:
        self.target_pos += np.array([dx, dy, dz])
        self.last_input_source = "gamepad"
        print(f"Gamepad control: dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f}")
```

**Control Flow with PyTorch Kinematics**:
```python
def apply_cartesian_control(self, current_joints):
    # Filter to first 7 joints (arm only)
    self.current_joint_pos = current_joints[:7]
    
    # Get current end-effector pose using PyTorch Kinematics FK
    current_ee_pos, current_ee_rot = self.get_current_ee_pose(self.current_joint_pos)
    
    # Calculate position error
    pos_error = self.target_pos - current_ee_pos

    # Construct 6D twist (3 position + 3 orientation)
    twist = np.zeros(6)
    twist[:3] = self.Kpos * pos_error / self.integration_dt
    
    # Orientation control
    current_rot = Rotation.from_matrix(current_ee_rot)
    desired_rot = Rotation.from_matrix(self.get_target_rotation_matrix())
    rot_error = (desired_rot * current_rot.inv()).as_rotvec()
    twist[3:] = self.Kori * rot_error / self.integration_dt
    
    # Get Jacobian using PyTorch Kinematics
    jac = self.compute_jacobian_pytorch(self.current_joint_pos)  # (6, 7)
    
    # Damped least squares + nullspace control
    diag = self.damping * np.eye(6)
    dq = jac.T @ np.linalg.solve(jac @ jac.T + diag, twist)
    
    # Nullspace control - drive towards home position
    jac_pinv = np.linalg.pinv(jac)
    N = np.eye(7) - jac_pinv @ jac  # Nullspace projection matrix
    dq_null = self.Kn * (self.home_pos - self.current_joint_pos)
    dq += N @ dq_null
    
    # Integrate to get new joint positions
    new_joints = self.current_joint_pos + dq * self.integration_dt
    
    # Return 8-dimensional control: 7 arm joints + gripper
    full_commands = np.zeros(8)
    full_commands[:7] = new_joints
    full_commands[7] = self.gripper_range[0] if self.gripper_state < 0.5 else self.gripper_range[1]
    
    return full_commands
```

#### 3. **MuJoCo Simulation Node** (`dora-mujoco`)
- **Input**: 8D joint commands (7 arm + 1 gripper) from enhanced controller
- **Processing**: Physics simulation, rendering, forward kinematics
- **Output**: Joint positions, velocities, sensor data

## Technical Implementation Details

### Control Modes Comparison

| Feature | Target Pose Control | Gamepad Control |
|---------|-------------------|-----------------|
| **Input Type** | Absolute positions | Incremental movements |
| **Update Rate** | Configurable | Real-time  |
| **Control Precision** | Exact target poses | Controller/gamepad positioning |
| **Gripper Control** | None | X/Y button control |
| **Speed Control** | Fixed | Variable (LB/RB buttons) |

### Incremental vs Absolute Control

**Target Pose Control** (Absolute):
```python
# Direct target assignment
self.target_pos = np.array([0.5, 0.2, 0.6])  # Go exactly here
```

**Gamepad Control** (Incremental):
```python
# Relative movement from current target
dx = gamepad_input * speed_scale * 0.1
self.target_pos += np.array([dx, dy, dz])     # Move relative to current
```

### Deadzone Implementation

```python
def apply_deadzone(self, value, deadzone=0.05):
    """Prevent controller drift by ignoring small inputs."""
    return 0.0 if abs(value) < deadzone else value
```

**Why needed**: Analog sticks rarely return exactly 0.0, causing unwanted drift.

## Extension Ideas

### Easy Extensions
1. **Add Orientation Control**:
   ```python
   # Use right stick X for yaw rotation
   dyaw = self.apply_deadzone(axes[2]) * self.speed_scale * 0.1
   self.target_rpy[2] += dyaw  # Update yaw angle
   ```

2. **Workspace Limits with FK Validation**:
   ```python
   # Validate target position is reachable using PyTorch Kinematics
   def validate_target_position(self, target_pos):
       # Use FK to check if any joint configuration can reach target
       # Could use IK solver to verify reachability
       pass
   ```

3. **Custom Button Mapping**:
   ```python
   # Load button mappings from config file
   button_config = {
       'gripper_close': 'X',
       'gripper_open': 'Y', 
       'speed_up': 'RB',
       'home_reset': 'START'
   }
   ```

### Advanced Extensions
1. **Force Feedback**: Rumble controller when approaching limits or singularities
2. **Multi-Robot Control**: Leverage PyTorch Kinematics batch processing for multiple arms
3. **Recording/Playback**: Record gamepad sessions with precise end-effector trajectories
4. **Learning Integration**: Use PyTorch's autodiff for learning-based assistance
5. **Collision Avoidance**: Integrate with [pytorch-volumetric](https://github.com/UM-ARM-Lab/pytorch_volumetric) for SDF-based collision checking

### Multi-Modal Input Example
```python
def main():
    # Controller can handle both gamepad and programmatic input
    for event in node:
        if event["id"] == "raw_control":
            # Gamepad input - incremental control
            controller.process_gamepad_input(raw_control)
        elif event["id"] == "target_pose":
            # Programmatic input - absolute positioning
            controller.process_target_pose(target_pose)
        
        # Same differential IK handles both input types seamlessly
        commands = controller.apply_cartesian_control(joint_positions)
```

## Troubleshooting

### "Gamepad not detected"
```bash
# Check if gamepad is connected
ls /dev/input/js*

# Test gamepad input
jstest /dev/input/js0
```

### "Robot doesn't respond to gamepad"
- Check that gamepad node is outputting `raw_control` data
- Verify controller is receiving gamepad events
- Ensure PyTorch Kinematics is properly initialized


### "Slow response / choppy movement"
- Enable GPU acceleration: check `torch.cuda.is_available()`
- Reduce gamepad polling rate if CPU-limited
- Profile FK/Jacobian computation times

### "Robot moves erratically with gamepad"
- Adjust deadzone: increase `self.deadzone = 0.1` for less sensitive sticks
- Reduce speed scale: lower `self.speed_scale = 0.2` for finer control
- Check for controller drift: test with `jstest`

## Performance Notes

- **Real-time Control**: PyTorch Kinematics enables smooth 100Hz gamepad response
- **GPU Acceleration**: Significant speedup for FK/Jacobian computation
- **Memory Efficiency**: Minimal memory overhead compared to dual MuJoCo
- **Scalability**: Could theoretically control multiple robots with one gamepad

## Next Steps

This gamepad control example demonstrates a complete teleoperation system with modern kinematics computation. Consider exploring:

- **Direct Robot Control**: Replace MuJoCo with real robot drivers (FrankaEmika SDK)
- **Advanced Input Devices**: 3D mice, haptic devices, VR controllers
- **Autonomous + Manual**: Blend autonomous behaviors with manual override
- **Multi-Modal Control**: Voice commands, gesture recognition, eye tracking
- **Learning-Based Assistance**: Use PyTorch for adaptive control behaviors
- **Collaborative Control**: Multiple operators controlling different aspects
