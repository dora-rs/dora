# 03. Gamepad Control

This example demonstrates real-time interactive control by connecting a gamepad to the Franka controller. It shows how to integrate the existing [`gamepad`](../../../node-hub/gamepad) node with robot-specific control logic.

## Learning Goals

- Integrate multiple input sources (gamepad + target poses)
- Process raw gamepad input into robot commands
- Implement real-time teleoperation
- Understand modular input processing

## Architecture

```
[Gamepad] → [Franka Controller] → [MuJoCo Sim] → [Outputs]
             ↑
    [Target Pose Publisher] (optional)
```

## Controls

- **Left Stick X**: Move along X-axis  
- **Left Stick Y**: Move along Y-axis
- **Right Stick Y**: Move along Z-axis
- **LB/RB**: Decrease/Increase movement speed
- **START**: Reset to home position
- **X Button**: Close gripper
- **Y Button**: Open gripper

## Running the Example

1. **Connect a gamepad** (Xbox/PlayStation controller)
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

## Key Features

### Enhanced Controller
The Franka controller now supports:
- **Dual Input Sources**: Both gamepad and target pose commands
- **Incremental Control**: Gamepad moves relative to current position
- **Speed Scaling**: Adjustable movement speed
- **Dead Zone**: Prevents controller drift

### Input Priority
- Gamepad input takes precedence when active
- Target pose commands work when gamepad is idle
- Smooth transitions between control modes

## Gamepad Mapping

The controller uses the raw gamepad data from the [`gamepad`](../../../node-hub/gamepad) node:

```json
{
  "axes": [stick_values...],    // Analog stick positions
  "buttons": [button_states...], // Button press states
  "mapping": {...}              // Button/axis name mappings
}
```

## Extension Ideas

- Add orientation control to right stick
- Implement force feedback
- Create custom button mappings

## Conclusion

You've now built a complete modular robot control system! This architecture demonstrates:

- **Separation of Concerns**: Simulation, control, and input are separate
- **Reusability**: Controller can work with different simulators
- **Extensibility**: Easy to add new input methods or robots
- **Maintainability**: Clear, testable components

This pattern scales to production robotics systems and can be adapted for any robot platform.