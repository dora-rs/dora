# 03. Gamepad Control

This example demonstrates real-time interactive control by connecting a gamepad to the controller. It builds upon the target pose control example by adding gamepad input processing for teleoperation of the robot arm.

## Controller Types

### 1. Basic Gamepad Control (`gamepad_control_basic.yml`)
- **Direct IK approach**: Uses simple IK solver for immediate position updates
- The movement fells jumpy

### 2. Advanced Gamepad Control (`gamepad_control_advanced.yml`)
- **Differential IK approach**: Smooth velocity-based control with Jacobian
- **Smooth**: Continuous motion interpolation
- **Use case**: Precise manipulation, smooth trajectories

## Gamepad Controls

- **D-pad Vertical**: Move along X-axis (forward/backward) 
- **D-pad Horizontal**: Move along Y-axis (left/right)
- **Right Stick Y**: Move along Z-axis (up/down)
- **LB/RB**: Decrease/Increase movement speed (0.1-1.0x scale)
- **START**: Reset to home position [0.4, 0.0, 0.3]

## Running the Examples

1. **Connect a gamepad** (Xbox/PlayStation controller via USB or Bluetooth)

### Basic Gamepad Control
```bash
cd gamepad_control
dora build gamepad_control_basic.yml
dora run gamepad_control_basic.yml
```

### Advanced Gamepad Control
```bash
cd gamepad_control
dora build gamepad_control_advanced.yml
dora run gamepad_control_advanced.yml
```

You should see:
1. Robot responds to gamepad input in real-time
2. **Basic**: Immediate position jumps based on gamepad input
3. **Advanced**: Smooth incremental movement with velocity control
4. Speed scaling with bumper buttons
5. Reset functionality with START button


### **Gamepad Node** (`gamepad`)
Built-in Dora node that interfaces with system gamepad drivers using Pygame.

```yaml
- id: gamepad
  build: pip install -e ../../../node-hub/gamepad
  path: gamepad
  outputs:
    - cmd_vel     # 6DOF velocity commands
    - raw_control # Raw button/stick states
  inputs:
    tick: dora/timer/millis/10  # 100Hz polling
```

- **`cmd_vel`**: 6DOF velocity array `[linear_x, linear_y, linear_z, angular_x, angular_y, angular_z]`
  - Generated from D-pad and analog stick positions
  - Continuous updates while controls are held
  
- **`raw_control`**: JSON format gamepad state


### **Gamepad Controller Scripts**

**Basic Controller (`gamepad_controller_ik.py`)**:
```
Gamepad Input → Target Position Update → IK Request → Joint Commands
```
- Updates target position incrementally based on gamepad
- Immediate position jumps

**Advanced Controller (`gamepad_controller_differential_ik.py`)**:
```
Gamepad Input → Target Position → Pose Error → Velocity → Joint Commands
```
- Continuous pose error calculation and velocity control
- Smooth interpolation between current and target poses
- Real-time Jacobian-based control

## Gamepad Mapping

The controller expects standard gamepad layout: (The mapping may change based on controller)

| Control | Function |
|---------|----------|
| D-pad Up/Down |  X-axis movement |
| D-pad Left/Right |  Y-axis movement |
| Right Stick Y |  Z-axis movement |
| Left Bumper |  Decrease speed |
| Right Bumper |  Increase speed |
| START button |  Reset position |

## Key Features

**Real-time Teleoperation:**
- Incremental position updates based on continuous input
- Immediate feedback through robot motion

**Speed Control:**
- Dynamic speed scaling (0.1x to 1.0x)
- Allows both coarse and fine manipulation

**Features:**
- Home position reset capability
- Bounded movement through incremental updates
- Working on collision avoidance

## Troubleshooting

**"Gamepad not responding"**
```bash
# Check if gamepad is connected
ls /dev/input/js*
# Test gamepad input  
jstest /dev/input/js0
# Grant permissions
sudo chmod 666 /dev/input/js0
```

**"Robot doesn't move with D-pad"**
- Check `cmd_vel` output: should show non-zero values when D-pad pressed
- Verify correct gamepad mapping for your controller model

**"Movement too fast/slow"**
- Use LB/RB buttons to adjust speed scale
- Modify `delta = cmd_vel[:3] * 0.03` scaling factor in code if required 
