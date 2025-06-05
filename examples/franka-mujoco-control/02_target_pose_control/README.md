# 02. Target Pose Control

This example demonstrates Cartesian space control by creating a dedicated Franka controller node that processes target pose commands and outputs joint commands using inverse kinematics.

## Running the Example

```bash
cd 02_target_pose_control
dora build target_pose_control.yml
dora run target_pose_control.yml
```

You should see:
1. Robot moves to predefined target poses automatically
2. Smooth Cartesian space motion with inverse kinematics
3. End-effector following target positions accurately

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

## How It Works

### Node Breakdown

#### 1. **Pose Publisher Node** (`pose_publisher.py`)
```python
class PosePublisher:
    def __init__(self):
        # Predefined sequence of target poses [x, y, z, roll, pitch, yaw]
        self.target_poses = [
            [0.5, 0.5, 0.3, 180.0, 0.0, 90.0],   # Position + RPY orientation
            [0.6, 0.2, 0.5, 180.0, 0.0, 45.0],   # Different orientation
            # ... more poses
        ]
```

**What it does**:
- Sends target poses every 10 seconds
- Cycles through predefined positions and orientations
- Can be replaced with path planning, user input, or any pose generation logic
- **Output**: `target_pose` array `[x, y, z, roll, pitch, yaw]`

#### 2. **Franka Controller Node** (`franka_controller.py`)

**Key Components**:

```python
class FrankaController:
    def __init__(self):
        # ⚠️ DUAL MUJOCO INSTANCE - loads separate model for kinematics
        self.model = load_robot_description("panda_mj_description", variant="scene")
        self.data = mujoco.MjData(self.model)
```

**What it does**:
- **Input**: Target poses `[x, y, z, roll, pitch, yaw]` and current joint positions
- **Processing**: Inverse kinematics using damped least squares with nullspace control
- **Output**: Joint position commands for the robot

**Control Algorithm**:
```python
def apply_cartesian_control(self, current_joints):
    # 1. Update internal model with current joint state
    self.data.qpos[:7] = current_joints[:7]
    mujoco.mj_forward(self.model, self.data)
    
    # 2. Get current end-effector pose from kinematics
    current_ee_pos = self.data.body(self.hand_id).xpos.copy()
    current_ee_rot = self.data.body(self.hand_id).xmat.reshape(3, 3)
    
    # 3. Calculate position and orientation errors
    pos_error = self.target_pos - current_ee_pos
    rot_error = (desired_rot * current_rot.inv()).as_rotvec()
    
    # 4. Create 6D twist vector [linear_vel, angular_vel]
    twist = np.zeros(6)
    twist[:3] = self.Kpos * pos_error / self.integration_dt
    twist[3:] = self.Kori * rot_error / self.integration_dt
    
    # 5. Compute Jacobian matrix
    jacp = np.zeros((3, self.model.nv))  # Position Jacobian
    jacr = np.zeros((3, self.model.nv))  # Rotation Jacobian
    mujoco.mj_jacBody(self.model, self.data, jacp, jacr, self.hand_id)
    jac = np.vstack((jacp[:, :7], jacr[:, :7]))
    
    # 6. Solve inverse kinematics with damped least squares
    diag = self.damping * np.eye(6)
    dq = jac.T @ np.linalg.solve(jac @ jac.T + diag, twist)
    
    # 7. Add nullspace control to prefer home position
    N = np.eye(7) - np.linalg.pinv(jac) @ jac  # Nullspace projection
    dq_null = self.Kn * (self.home_pos - current_joints[:7])
    dq += N @ dq_null
    
    # 8. Integrate to get new joint positions
    new_joints = current_joints[:7] + dq * self.integration_dt
```

#### 3. **MuJoCo Simulation Node** (`dora-mujoco`)
- **Input**: Joint commands from controller
- **Processing**: Physics simulation, rendering, forward kinematics
- **Output**: Joint positions, velocities, sensor data

## Technical Implementation Details

**The Problem**: This implementation runs **TWO separate MuJoCo instances**:

1. **Main Simulation** (`dora-mujoco` node): Handles physics, rendering, what you see
2. **Controller Kinematics** (`franka_controller.py`): Separate headless instance for inverse kinematics

```python
# In franka_controller.py - SEPARATE MUJOCO INSTANCE
self.model = load_robot_description("panda_mj_description", variant="scene")
self.data = mujoco.MjData(self.model)  # Independent physics state!
```

**Why I Did This**:
- **Accurate Kinematics**: MuJoCo provides exact forward kinematics and Jacobian computation
- **Smooth Control**: Custom geometric kinematics often have numerical issues
<span style="color: #888888; opacity: 1.0;">I got Lazy 🙃: Easier than implementing analytical kinematics from scratch</span>

#### Alternative Approaches: Pure Geometric Kinematics

Replace the dual MuJoCo with analytical kinematics:

```python
class FrankaController:
    def __init__(self):
        # No MuJoCo model - just DH parameters
        self.dh_params = self._get_franka_dh_parameters()
    
    def forward_kinematics(self, joint_angles):
        """Compute FK using DH parameters"""
        # Analytical forward kinematics implementation
        
    def compute_jacobian(self, joint_angles):
        """Analytical or numerical differentiation"""
```

**Pros**: Single source of truth
**Cons**: More complex to implement, potential numerical issues

### RPY Orientation Control Issues

**The Problem**: Roll-Pitch-Yaw (RPY) orientation control has limitations:

```python
target_pose = [0.5, 0.2, 0.6, 180.0, 0.0, 90.0]  # x, y, z, roll, pitch, yaw
```

**Issues**:
- **Gimbal Lock**: Certain orientations become unreachable
- **Order Dependency**: ZYX Euler angle convention may not match user expectations

**Better Alternatives**:
- **Quaternions**: `[x, y, z, qw, qx, qy, qz]` - no singularities (but not at all intuitive for humans)

## Key Controller Features

- **Cartesian Control**: Independent position and orientation control
- **Joint Limits**: Respects Franka's mechanical constraints  
- **Nullspace Control**: Returns to preferred joint configuration when possible
- **Damped Least Squares**: Handles near-singular configurations gracefully


## Production Recommendations

For real robot deployment:
1. Replace dual MuJoCo with analytical kinematics or robot manufacturer's libraries
2. Use quaternion orientation representation  
3. Add collision checking and joint limit monitoring
4. Implement proper error handling and safety stops
5. Add force/torque control capabilities

## Extensions

This example can be extended with:
- **Path Planning**: Replace pose publisher with trajectory generation
- **Obstacle Avoidance**: Add collision checking to controller
- **Force Control**: Implement hybrid position/force control
- **Multi-Robot**: Coordinate multiple robot arms
- **Real Robot**: Replace MuJoCo with actual robot drivers

## Troubleshooting

**"Controller gets out of sync after GUI reset"**
- This is due to dual MuJoCo instances - the controller's internal model doesn't know about GUI resets
- Restart the entire pipeline after using GUI reset

**"Robot moves erratically with certain orientations"**  
- Certain RPY orientations sometimes cause issues
- Try quaternion representation or avoid problematic orientations