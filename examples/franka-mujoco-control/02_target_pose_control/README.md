# 02. Target Pose Control

This example adds robot-specific control logic by creating a dedicated Franka controller node that processes target pose commands and outputs joint commands.

## Learning Goals

- Implement Cartesian space control with inverse kinematics
- Create robot-specific controller nodes
- Send programmatic target pose commands

## Architecture

```
[Pose Publisher] → [Franka Controller] → [MuJoCo Sim] → [Outputs]
     ↓                      ↓                  ↓
[Target Poses]     [Joint Commands]    [Joint States]
```

## Running the Example

```bash
cd 02_target_pose_control
dora build target_pose_control.yml
dora run target_pose_control.yml
```

You should see:
1. Robot moves to predefined target poses automatically
2. Smooth Cartesian space motion with inverse kinematics

## Interactive Control

While the simulation is running, you can send custom poses:

```python
# In another terminal
python3 -c "
import pyarrow as pa
from dora import Node
import time

node = Node()

# Move to position (0.5, 0.2, 0.6) with downward orientation
target_pose = [0.5, 0.2, 0.6, 180.0, 0.0, 90.0]  # x, y, z, roll, pitch, yaw

node.send_output(
    'target_pose',
    pa.array(target_pose, type=pa.float64()),
    metadata={'timestamp': time.time()}
)
"
```

## Key Components

### Franka Controller Node
- **Input**: Target poses `[x, y, z, roll, pitch, yaw]`
- **Output**: Joint position commands  
- **Algorithm**: Damped least squares inverse kinematics with nullspace control

### Pose Publisher Node
- Sends predefined target poses in sequence
- Demonstrates programmatic control
- Can be replaced with your own pose generation logic or pose path planning

## Controller Features

- **Cartesian Control**: Position and orientation control
- **Joint Limits**: Respects Franka's joint constraints
- **Nullspace Control**: Returns to preferred joint configuration
- **Smooth Motion**: Velocity-limited for safety