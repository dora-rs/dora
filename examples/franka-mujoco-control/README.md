# Franka MuJoCo Control Tutorial

This comprehensive tutorial demonstrates how to build a modular robot control system using Dora's dataflow architecture with the [`dora-mujoco`](../../node-hub/dora-mujoco) simulation node and robot-specific control logic.

## Tutorial Structure

### [01. Basic Simulation](01_basic_simulation/)
Load and run a Franka Panda simulation using the `dora-mujoco` node.
- Learn the fundamentals of MuJoCo simulation in Dora
- Understand the simulation node architecture
- See how robot descriptions are loaded automatically

### [02. Target Pose Control](02_target_pose_control/) 
Add robot-specific control logic with programmatic pose commands.
- Implement Cartesian space control with inverse kinematics
- Create a dedicated Franka controller node
- Send target poses programmatically

### [03. Gamepad Control](03_gamepad_control/)
Connect a gamepad for real-time interactive control.
- Integrate with the existing `gamepad` node
- Process raw gamepad input into robot commands
- Demonstrate real-time teleoperation


## Quick Start

```bash
cd examples/franka-mujoco-control

# 1. Basic simulation
cd 01_basic_simulation
dora build basic.yml
dora run basic.yml

# 2. Target pose control  
cd ../02_target_pose_control
dora build target_pose_control.yml
dora run target_pose_control.yml

# 3. Gamepad control
cd ../03_gamepad_control
dora build gamepad_control.yml
dora run gamepad_control.yml
```

## Next Steps

After completing this tutorial, you'll understand how to:
- Build modular robotics applications with Dora
- Create robot-specific controllers
- Design scalable dataflow architectures

