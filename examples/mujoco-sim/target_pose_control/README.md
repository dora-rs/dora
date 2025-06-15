# 02. Target Pose Control

This example demonstrates Cartesian space control using two different approaches: **Direct Inverse Kinematics(IK)**(Basic) and **Differential IK**(advance). Both create a Controller node that processes target pose commands and outputs joint commands using **dora-pytorch-kinematics**.

## Controller Types

### 1. Direct IK Controller (`control.yml`)
- **Simple approach**: Directly passes target pose to IK solver
- **Fast**: Single IK computation per target pose

### 2. Differential IK Controller (`control_advanced.yml`)  
- **Smooth approach**: Uses pose error feedback and Jacobian-based control
- **Continuous**: Interpolates smoothly between current and target poses
- **Use case**: Smooth trajectories

## Running the Examples

### Direct IK Control
```bash
cd target_pose_control
dora build control.yml
dora run control.yml
```

### Differential IK Control
```bash
cd target_pose_control
dora build control_advanced.yml
dora run control_advanced.yml
```

You should see:
1. Robot moves to predefined target poses automatically
2. **Direct IK**: Immediate jumps to target poses
3. **Differential IK**: Smooth Cartesian space motion with continuous interpolation
4. End-effector following target positions accurately


### Nodes

#### 1. **Pose Publisher Script** (`pose_publisher.py`)
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

- Sends target poses every 5 seconds
- Cycles through predefined positions and orientations
- Can be replaced with path planning, user input, or any pose generation logic
- Outputs `target_pose` array `[x, y, z, roll, pitch, yaw]` 

#### 2. **Controller Scripts**

##### Direct IK Controller (`controller_ik.py`)

**How it works:**
1. **Target Input**: Receives new target pose `[x, y, z, roll, pitch, yaw]`
2. **IK Request**: Sends target pose directly to `dora-pytorch-kinematics`
3. **Joint Solution**: Receives complete joint configuration for target pose
4. **Direct Application**: Passes IK solution directly as joint commands to robot *(sometimes for certain target pose there is no IK solution)*

**Advantages:**
- Simple and fast0
- Minimal computation
- Direct pose-to-joint mapping

**Disadvantages:**
- Sudden jumps between poses
- No trajectory smoothing
- May cause joint velocity spikes 

##### Differential IK Controller (`controller_differential_ik.py`)

**How it works:**
1. **Pose Error Calculation**: Computes difference between target and current end-effector pose
2. **Velocity Command**: Converts pose error to desired end-effector velocity using PD control:
   ```python
   pos_error = target_pos - current_ee_pos
   twist[:3] = Kpos * pos_error / integration_dt  # Linear velocity
   twist[3:] = Kori * rot_error / integration_dt  # Angular velocity
   ```
3. **Jacobian Inverse**: Uses robot Jacobian to map end-effector velocity to joint velocities:
   ```python
   # Damped least squares to avoid singularities
   dq = J^T @ (J @ J^T + λI)^(-1) @ twist
   ```
4. **Interpolation**: Integrates joint velocities to get next joint positions:
   ```python
   new_joints = current_joints + dq * dt
   ```
5. **Nullspace Control** (optional): Projects secondary objectives (like joint limits avoidance) into the nullspace


**Advantages:**
- Smooth, continuous motion
- Velocity-controlled approach
- Handles robot singularities 
- Real-time reactive control

<!-- **Jacobian Role:**
- **Forward Kinematics**: Maps joint space to Cartesian space
- **Jacobian Matrix**: Linear mapping between joint velocities and end-effector velocities
- **Inverse Mapping**: Converts desired end-effector motion to required joint motions
- **Singularity Handling**: Damped least squares prevents numerical instability near singularities -->

##### 3. **PyTorch Kinematics Node** (`dora-pytorch-kinematics`)

A dedicated kinematics computation node that provides three core robotic functions:

```yaml
- id: pytorch_kinematics
  build: pip install -e ../../../node-hub/dora-pytorch-kinematics
  path: dora-pytorch-kinematics
  inputs:
    ik_request: controller/ik_request           # For inverse kinematics
    fk_request: controller/fk_request           # For forward kinematics  
    jacobian_request: controller/jacobian_request  # For Jacobian computation
  outputs:
    - ik_request      # Joint solution for target pose
    - fk_request      # End-effector pose for joint configuration
    - jacobian_request # Jacobian matrix for velocity mapping
  env:
    URDF_PATH: "../URDF/franka_panda/panda.urdf"
    END_EFFECTOR_LINK: "panda_hand"
    TRANSFORM: "0. 0. 0. 1. 0. 0. 0."
```

1. **Inverse Kinematics (IK)**
   - **Input**: Target pose `[x, y, z, roll, pitch, yaw]` or `[x, y, z, qw, qx, qy, qz]` + current joint state
   - **Output**: Complete joint configuration to achieve target pose
   - **Use case**: Convert Cartesian target to joint angles

2. **Forward Kinematics (FK)**  
   - **Input**: Joint positions array
   - **Output**: Current end-effector pose `[x, y, z, qw, qx, qy, qz]`
   - **Use case**: Determine end-effector position from joint angles

3. **Jacobian Computation**
   - **Input**: Current joint positions  
   - **Output**: 6×N Jacobian matrix (N = number of joints)
   - **Use case**: Map joint velocities to end-effector velocities

**Configuration:**
- **URDF_PATH**: Robot model definition file
- **END_EFFECTOR_LINK**: Target link for pose calculations
- **TRANSFORM**: Optional transform offset (position + quaternion wxyz format)

**Usage in Controllers:**
- **Direct IK**: Uses only `ik_request` → `ik_result`
- **Differential IK**: Uses `fk_request` → `fk_result` and `jacobian_request` → `jacobian_result`

#### 4. **MuJoCo Simulation Node** (`dora-mujoco`)

- **Process**: Physics simulation, dynamics integration, rendering
- **Output**: Joint positions, velocities, sensor data

## References

This controller design draws inspiration from the kinematic control strategies presented in [mjctrl](https://github.com/kevinzakka/mjctrl), specifically the [differntial ik control example](https://github.com/kevinzakka/mjctrl/blob/main/diffik.py).

The URDF model for the robotic arms was sourced from the [PyBullet GitHub repository](https://github.com/bulletphysics/bullet3/tree/master/examples/pybullet/gym/pybullet_data). Or you could google search the robot and get its urdf.