# MuJoCo Sim Tutorial

This comprehensive tutorial demonstrates how to build a robot control system using Dora with the `dora-mujoco` simulation node and control logic.

## Tutorial Structure

### [01. Basic Simulation](basic_simulation/)
Load a robot in simulation using the `dora-mujoco` node.
- Learn the fundamentals of MuJoCo simulation in Dora
- Understand the simulation node architecture
- See how robot descriptions are loaded automatically

### [02. Target Pose Control](target_pose_control/) 
Add control logic with pose commands as target.
- Implement Cartesian space control.
- Create generic controller node that is able to control any robotic arm by using `dora-pytorch-kinematics`

### [03. Gamepad Control](gamepad_control/)
Connect a gamepad for real-time interactive control.
- Integrate with dora's `gamepad` node
- Demonstrate real-time teleoperation
