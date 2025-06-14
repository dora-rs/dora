# 02. Target Pose Control

This example demonstrates Cartesian space control by creating a Controller node that processes target pose commands and outputs joint commands using inverse kinematics using **dora-pytorch-kinematics**.

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

- Sends target poses every 10 seconds
- Cycles through predefined positions and orientations
- Can be replaced with path planning, user input, or any pose generation logic
- Outputs `target_pose` array `[x, y, z, roll, pitch, yaw]` 

#### 2. **Controller Script** (`controller.py`)
```yaml
  - id: controller
    path: nodes/controller.py
    inputs:
      joint_positions: mujoco_sim/joint_positions
      joint_velocities: mujoco_sim/joint_velocities
      target_pose: pose_publisher/target_pose
      fk_result: pytorch_kinematics/fk_request
      jacobian_result: pytorch_kinematics/jacobian_request
    outputs:
      - joint_commands
      - fk_request
      - jacobian_request
```
- **Input**: Takes target poses `[x, y, z, roll, pitch, yaw]` and current joint positions from `dora-mujoco` 
- **Processing**: Differential inverse kinematics using `dora-pytorch-kinematics` to calculate actuator commands
- **Output**: Control/Actuator commands

#### 3. **PyTorch Kinematics Node** (`dora-pytorch-kinematics`)


```yaml
- id: pytorch_kinematics
    build: pip install -e ../../../node-hub/dora-pytorch-kinematics
    path: dora-pytorch-kinematics
    inputs:
      fk_request: controller/fk_request
      jacobian_request: controller/jacobian_request
    outputs:
      - fk_request  
      - jacobian_request  
    env:
      URDF_PATH: "path to the robot urdf" # URDF is used to create the kinematics model for the robot 
      END_EFFECTOR_LINK: "name of the end effector"
      TRANSFORM: "0. 0. 0. 1. 0. 0. 0."  # Pytorch kinematics uses wxyz format. Robot transform from world frame

```

 Joint states performs Forward Kinematics, and returns End-effector pose along with jacobian matrix

#### 4. **MuJoCo Simulation Node** (`dora-mujoco`)

```yaml

  - id: mujoco_sim
    build: pip install -e ../../../node-hub/dora-mujoco
    path: dora-mujoco
    inputs:
      tick: dora/timer/millis/2
      control_input: controller/joint_commands
    outputs:
      - joint_positions
      - joint_velocities 
      - actuator_controls
      - sensor_data
    env:
      MODEL_NAME: "panda" # Name of the robot you want to load
```
- **Input**: Joint commands from controller
- **Processing**: Physics simulation, rendering
- **Output**: Joint positions, velocities, sensor data