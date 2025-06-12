# 02. Target Pose Control

This example demonstrates Cartesian space control by creating a dedicated Franka controller node that processes target pose commands and outputs joint commands using inverse kinematics with **PyTorch Kinematics**.

## Running the Example

```bash
cd 02_target_pose_control
dora build target_pose_control_pytorch.yml
dora run target_pose_control_pytorch.yml
```

You should see:
1. Robot moves to predefined target poses automatically
2. Smooth Cartesian space motion with differential inverse kinematics
3. End-effector following target positions accurately
4. GPU-accelerated kinematics computation (if CUDA available)

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

#### 2. **Franka Controller Node** (`franka_controller_pytorch.py`)

**Key Components**:

```python
class FrankaController:
    def __init__(self):
        urdf_path = "path to the file/panda.urdf"
        with open(urdf_path, 'rb') as f:
            urdf_content = f.read()
        
        self.chain = pk.build_serial_chain_from_urdf(urdf_content, "panda_hand")
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.chain = self.chain.to(device=self.device)
```

**What it does**:
- **Input**: Target poses `[x, y, z, roll, pitch, yaw]` and current joint positions from MuJoCo
- **Processing**: Differential inverse kinematics using PyTorch Kinematics
- **Output**: Joint position commands for the robot

**Control Algorithm (Differential IK)**:
```python
def apply_differential_ik_control(self):
    # 1. Get current end-effector pose using PyTorch Kinematics FK
    current_ee_pos, current_ee_rot = self.get_current_ee_pose(self.current_joint_pos)
    
    # 2. Calculate position and orientation errors
    pos_error = self.target_pos - current_ee_pos
    rot_error = (desired_rot * current_rot.inv()).as_rotvec()
    
    # 3. Create 6D twist vector [linear_vel, angular_vel]
    twist = np.zeros(6)
    twist[:3] = self.Kpos * pos_error / self.integration_dt
    twist[3:] = self.Kori * rot_error / self.integration_dt
    
    # 4. Compute Jacobian using PyTorch Kinematics
    jac = self.compute_jacobian_pytorch(self.current_joint_pos)  # (6, 7)
    
    # 5. Solve differential IK with damped least squares
    diag = self.damping * np.eye(6)
    dq = jac.T @ np.linalg.solve(jac @ jac.T + diag, twist)
    
    # 6. Add nullspace control to prefer home position
    N = np.eye(7) - np.linalg.pinv(jac) @ jac  # Nullspace projection
    dq_null = self.Kn * (self.home_pos - self.current_joint_pos)
    dq += N @ dq_null
    
    # 7. Integrate to get new joint positions
    new_joints = self.current_joint_pos + dq * self.integration_dt
```

#### 3. **MuJoCo Simulation Node** (`dora-mujoco`)
- **Input**: Joint commands from controller
- **Processing**: Physics simulation, rendering, forward kinematics
- **Output**: Joint positions, velocities, sensor data

## Technical Implementation Details

- **Main Simulation** (`dora-mujoco` node): Physics, rendering, joint state
- **Controller** (`franka_controller_pytorch.py`): PyTorch Kinematics for FK/Jacobian 
- **Single source of truth**: MuJoCo simulation provides all joint states / sensor feedback in case of hardware

```python
class FrankaController:
    def __init__(self):
        # Load kinematics model for computation only
        self.chain = pk.build_serial_chain_from_urdf(urdf_content, "panda_hand")
        self.chain = self.chain.to(device=self.device)  # GPU acceleration
        
    def get_current_ee_pose(self, joint_angles):
        """PyTorch Kinematics FK"""
        q = torch.tensor(joint_angles, device=self.device, dtype=torch.float32).unsqueeze(0)
        tf = self.chain.forward_kinematics(q)
        # ... extract position and rotation
```

### Key Advantages

**Performance Benefits**:
- **GPU Acceleration**: PyTorch Kinematics can leverage CUDA for faster computation
- **Optimized Gradients**: Built-in automatic differentiation


**Control Benefits**:
- **Differential IK**: Smoother motion than discrete IK solving
- **Nullspace Control**: Avoids joint limits and singularities
- **Real-time Performance**: currently 500Hz control loops

### Differential vs. Analytical IK

**Differential IK** (Current Implementation):
```python
# Compute small joint movements based on end-effector error
dq = jac.T @ np.linalg.solve(jac @ jac.T + diag, twist)
new_joints = current_joints + dq * dt
```

**Analytical IK** (Alternative):
```python
# Solve for exact joint configuration to reach target [sometimes exact solution is not available can result in `None return`]
ik_solver = pk.PseudoInverseIK(chain, ...)
solution = ik_solver.solve(target_transform)
```

**Why Differential IK**:
- **Smoother motion**: Continuous trajectory without jumps
- **Better convergence**: Less likely to get stuck in local minima
- **Singularity handling**: Graceful behavior near workspace boundaries

### PyTorch Kinematics Features Used

1. **Forward Kinematics**: `chain.forward_kinematics(q)`
2. **Jacobian Computation**: `chain.jacobian(q)`
3. **GPU Support**: `.to(device=device)`
4. **Batch Processing**: Handle multiple configurations simultaneously
5. **Automatic Differentiation**: Could enable learning-based control

## Production Recommendations

For real robot deployment:
1. **Kinematics Library**: Currently the urdf is used to create the model for pytorch, for you robot you need to make it manually
2. **Use Quaternions**: Replace RPY with quaternion orientation representation
3. **Add Safety Monitors**: Joint limit monitoring, collision checking
4. **Real Robot Interface**: Replace MuJoCo with actual robot drivers
5. **Advanced Control**: Force/torque control, compliant motion

## Extensions

This example can be extended with:
- **Learning-Based Control**: Use PyTorch's autodiff for learned components
- **Multi-Robot Coordination**: Leverage GPU parallel processing
- **Advanced IK Solvers**: Try different PyTorch Kinematics IK algorithms
- **Collision Avoidance**: Integrate with [pytorch-volumetric](https://github.com/UM-ARM-Lab/pytorch_volumetric) for SDF queries
- **Real Robot**: Replace MuJoCo with actual robot drivers

## Troubleshooting

**"PyTorch Kinematics not found"**
```bash
pip install pytorch-kinematics
```

**"CUDA out of memory"**
- Set `device = "cpu"` in controller initialization
- Reduce batch sizes if using advanced features

**"Robot moves erratically"**
- Check joint limits are correctly applied
- Verify URDF file path is correct
- Try reducing control gains if oscillating

**"Controller slower than expected"**
- Ensure PyTorch is using GPU: check `torch.cuda.is_available()`
- Profile the forward kinematics and Jacobian computation times

## Performance Notes

- **GPU Acceleration**: ~10x speedup for kinematics computation with CUDA
- **Memory Usage**: Minimal - only loads kinematic chain, not full physics
- **Scalability**: Can handle multiple robot arms with batch processing
