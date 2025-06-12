# Franka Real-Time Collision Avoidance Controller

Franka Panda robot control with gamepad input and real-time self-collision avoidance using repulsive forces. The robot provides smooth, collision-aware motion.

## Features

- **Real-Time Gamepad Control**: Intuitive joystick control for precise robot movement
- **Self-Collision Avoidance**: Continuous collision detection and avoidance using repulsive forces
- **PyTorch Kinematics**: GPU-accelerated forward/inverse kinematics for high performance
- **Smooth Motion**: Natural robot behavior with collision-aware motion planning

## Controls

### Gamepad Layout
- **Left Stick X/Y**: Move end-effector in X/Y plane
- **Right Stick Y**: Move end-effector up/down (Z axis)
- **X Button**: Close gripper
- **Y Button**: Open gripper
- **LB/RB**: Decrease/Increase movement speed (0.1x - 1.0x)
- **START**: Reset to home position with downward orientation

## Collision Avoidance System

The system implements a sophisticated collision avoidance algorithm:

### Real-Time Detection
- **Continuous Monitoring**: Checks for potential self-collisions every control cycle
- **Geometric Collision Pairs**: Monitors 18 non-adjacent link pairs for potential collisions
- **Distance-Based Forces**: Generates repulsive forces based on proximity to collision

### Repulsive Force Model
```python
# Force magnitude calculation
if distance < min_safe_distance:
    force = collision_gain * (1/distance - 1/min_safe_distance)
else:
    force = 0  # No force outside influence zone
```

### Key Parameters
- **`min_link_distance`**: 0.02m - Minimum safe distance between robot links
- **`collision_force_gain`**: 1000.0 - Strength of repulsive forces
- **`collision_influence_distance`**: 0.05m - Distance at which collision avoidance activates

## Installation

### Prerequisites
```bash
# Install required Python packages
pip install pytorch-kinematics scipy numpy torch pyarrow dora-rs
```

### Running the System
```bash
# Navigate to the project directory
cd /path/to/dora-rs/examples/franka-mujoco-control/04_gamepad_collision_avoidance/

# Start the Dora runtime
dora up

# Launch the collision avoidance system
dora start collision_avoidance.yml
```

## System Architecture

<!-- ### Node Communication Flow
```
┌─────────────┐    raw_control      ┌─────────────────────────────┐
│   gamepad   │ ──────────────────▶ │ franka_collision_controller │
└─────────────┘                     │                             │
                                    │  • Processes gamepad        │
┌─────────────┐   joint_positions   │  • Calculates IK/FK         │
│ mujoco_sim  │ ──────────────────▶ │  • Applies collision        │
│             │                     │    avoidance                │
│             │ ◀────────────────── │  • Outputs joint cmds       │
└─────────────┘   joint_commands    └─────────────────────────────┘
``` -->

### Collision Detection Pipeline
1. **Forward Kinematics**: Calculate all link positions using PyTorch
2. **Distance Calculation**: Check distances between all collision pairs
3. **Force Generation**: Generate repulsive forces for close pairs
4. **Force Integration**: Convert Cartesian forces to joint space
5. **Motion Blending**: Combine tracking and collision avoidance motions

## Configuration

### Controller Parameters
```python
# Control gains
Kpos = 0.95          # Position control gain
Kori = 0.95          # Orientation control gain
max_angvel = 0.785   # Maximum joint velocity (rad/s)

# Collision avoidance
min_link_distance = 0.02           # 2cm minimum safe distance
collision_force_gain = 1000.0      # Repulsive force strength
collision_influence_distance = 0.05 # 5cm activation distance
```

### Link Geometry Configuration
The system models each robot link as a cylinder with specified radius and offset:
```python
link_geometries = {
    'panda_link1': {'radius': 0.08, 'offset': [0, 0, 0.075]},
    'panda_link2': {'radius': 0.08, 'offset': [0, 0, 0]},
    # ... etc for all 7 links + hand
}
```

## Troubleshooting

### Common Issues
1. **Robot Not Moving**: Check gamepad connection and button mappings
2. **Jerky Motion**: Reduce collision force gains
3. **GPU Errors**: Ensure CUDA-compatible PyTorch installation
4. **Joint Limits**: Robot automatically respects joint limits and stops at boundaries

### Debug Information
The system provides real-time collision status:
```bash
# Console output shows active collisions
Active collisions: panda_link1<->panda_link4: 0.03m; panda_link2<->panda_link5: 0.04m
Collision forces cleared - normal motion resumed
```

## Performance Notes

- **Control Frequency**: 500Hz for smooth, responsive control
- **Collision Detection**: Runs every control cycle (no artificial delays)
- **GPU Utilization**: Automatic GPU acceleration when CUDA available
- **Memory Usage**: Efficient tensor operations minimize memory footprint

## Safety Features

- **Joint Limit Protection**: Automatic velocity limiting near joint boundaries
- **Velocity Clamping**: Maximum joint velocities enforced at all times
- **Self-Collision Avoidance**: Repulsive force approach to avoid collision 
