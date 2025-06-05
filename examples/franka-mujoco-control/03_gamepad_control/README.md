# 03. Gamepad Control

This example demonstrates real-time interactive control by connecting a gamepad to the Franka controller. It builds upon the target pose control example by adding gamepad input processing while maintaining the same dual MuJoCo architecture. Shows how to integrate multiple input sources and implement teleoperation.

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

**Key Enhancement**: Extends the target pose controller with gamepad input processing.

```python
class EnhancedFrankaController:
    def __init__(self):
        # ⚠️ SAME DUAL MUJOCO ISSUE as target pose control
        self.model = load_robot_description("panda_mj_description", variant="scene")
        self.data = mujoco.MjData(self.model)
        
        # Gamepad-specific parameters
        self.speed_scale = 0.5      # Movement speed multiplier
        self.deadzone = 0.05        # Joystick deadzone threshold
        self.gripper_range = [0, 255]  # Gripper control range
```

**Gamepad Input Processing**:
```python
def process_gamepad_input(self, raw_control):
    axes = raw_control["axes"]
    buttons = raw_control["buttons"]
    
    # 1. Button handling
    if buttons[9]:  # START button
        # Reset robot to home position
        self.data.qpos[:7] = self.home_pos
        self.target_pos = self.data.body(self.hand_id).xpos.copy()
    
    # 2. Gripper control
    if buttons[0]:      # X button - Close gripper
        self.data.ctrl[self.gripper_actuator] = self.gripper_range[0]
    elif buttons[3]:    # Y button - Open gripper  
        self.data.ctrl[self.gripper_actuator] = self.gripper_range[1]
    
    # 3. Speed scaling
    if buttons[4]: self.speed_scale = max(0.1, self.speed_scale - 0.1)  # LB
    if buttons[5]: self.speed_scale = min(1.0, self.speed_scale + 0.1)  # RB
    
    # 4. Incremental position control
    dx = self.apply_deadzone(axes[0]) * self.speed_scale * 0.1
    dy = -self.apply_deadzone(axes[1]) * self.speed_scale * 0.1  # Inverted Y
    dz = -self.apply_deadzone(axes[3]) * self.speed_scale * 0.1  # Right stick Y
    
    # 5. Update target position incrementally
    if abs(dx) > 0 or abs(dy) > 0 or abs(dz) > 0:
        self.target_pos += np.array([dx, dy, dz])
```

**Control Flow**:
```python
def apply_cartesian_control(self, current_joints):
    # Same inverse kinematics as target pose control
    # BUT now returns 8D commands: [7 arm joints + 1 gripper]
    
    # ... (same IK algorithm as before) ...
    
    # Return 8-dimensional control: 7 arm joints + gripper
    full_commands = np.zeros(8)
    full_commands[:7] = new_joints
    full_commands[7] = self.data.ctrl[self.gripper_actuator]  # Gripper state
    
    return full_commands
```

#### 3. **MuJoCo Simulation Node** (`dora-mujoco`)
Same as target pose control - handles physics, rendering, and outputs joint states.

## Technical Implementation Details

### Control Modes Comparison

| Feature | Target Pose Control | Gamepad Control |
|---------|-------------------|-----------------|
| **Input Type** | Absolute positions | Incremental movements |
| **Update Rate** | Configurable | Real-time (100Hz) |
| **Control Precision** | Exact target poses | Human-guided positioning |
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

2. **Workspace Limits**:
   ```python
   # Prevent robot from leaving safe workspace
   workspace_bounds = {
       'x': [0.2, 0.8], 'y': [-0.5, 0.5], 'z': [0.1, 0.7]
   }
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
1. **Force Feedback**: Rumble controller when approaching limits (honestly not sure how to do this, but should be possible and fun)
2. **Multi-Robot**: Control multiple arms with different controllers
3. **Recording/Playback**: Record gamepad sessions for replay (Data Collection)

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

## Next Steps

This gamepad control example demonstrates a complete teleoperation system. Consider exploring:

- **Direct Robot Control**: Replace MuJoCo with real robot drivers
- **Advanced Input Devices**: 3D mice, haptic devices, VR controllers
- **Autonomous + Manual**: Blend autonomous behaviors with manual override
- **Multi-Modal Control**: Voice commands, gesture recognition, eye tracking

<span style="color: #888888; opacity: 1.0;">All of these are really cool addons that can be implemented</span>