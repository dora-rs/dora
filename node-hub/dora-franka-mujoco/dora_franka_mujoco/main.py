"""MuJoCo Franka simulation node for Dora.

This module provides a Dora node that simulates a Franka robot using the MuJoCo physics engine.
It handles raw gamepad input for robot control and target pose commands.
"""

import os
import json
import numpy as np
import mujoco
import mujoco.viewer
import pyarrow as pa
from dora import Node
from scipy.spatial import transform
from .mesh_downloader import check_mesh_files_exist, ensure_meshes

class RobotController:
    """Handles robot control modes and mappings."""
    def __init__(self):
        # Control parameters
        self.dt = 0.002  # MuJoCo timestep
        self.integration_dt = 0.1  
        self.damping = 1e-4  
        self.Kpos = 0.95  
        self.Kori = 0.95 
        self.Kn = np.array([10.0, 10.0, 10.0, 10.0, 5.0, 5.0, 5.0])  # Nullspace gains
        self.max_angvel = 0.785  # Maximum joint velocity (rad/s)
        self.speed_scale = 0.5
        self.deadzone = 0.05  # Joystick deadzone
        self.gripper_range = [0, 255]
        self.gripper_state = 0.0  # (0=closed, 1=open)
        self.target_rot_z = 0.0 
        self.rotation_scale = 0.2
        self.last_input_source = None
        
        # Default RPY values for downward-facing gripper (in degrees)
        self.target_rpy = [180.0, 0.0, 90.0]

    def set_target_orientation_rpy(self, roll_deg, pitch_deg, yaw_deg):
        """Set target orientation using roll, pitch, yaw in degrees."""
        self.target_rpy = [roll_deg, pitch_deg, yaw_deg]

    def get_target_rotation_matrix(self):
        """convert RPY to rotation matrix."""
        roll_rad, pitch_rad, yaw_rad = np.radians(self.target_rpy)
        desired_rot = transform.Rotation.from_euler('ZYX', [yaw_rad, pitch_rad, roll_rad])
        return desired_rot.as_matrix()

    def initialize(self, model, data):
        """Initialize controller with model specific parameters."""
        # Get end effector (hand) body ID
        self.hand_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "hand")  
        
        # Get gripper actuator IDs
        self.gripper_actuator = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "actuator8")
        
        # Initial end effector position and orientation to set as home pose
        self.home_pos = data.qpos.copy()
        self.target_pos = data.body(self.hand_id).xpos.copy()

        # Set gravity
        # model.opt.gravity[2] = 0.0
        model.opt.gravity[2] = -9.81

    def reset_pose(self, model, data):
        """Reset robot to home position."""
        data.qpos[:] = self.home_pos
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)
        
        # Reset target position and Orientation
        self.target_pos = data.body(self.hand_id).xpos.copy()
        # self.initial_rot = transform.Rotation.from_matrix(data.body(self.hand_id).xmat.reshape(3, 3))

        print("Robot reset to home position")

    def process_target_pose(self, target_pose, model, data):
        """Process target pose input (x, y, z, roll, pitch, yaw)."""

        self.target_pos = np.array(target_pose[:3])
        # If 6 elements provided, update orientation as well
        if len(target_pose) >= 6:
            roll, pitch, yaw = target_pose[3:6]
            self.set_target_orientation_rpy(roll, pitch, yaw)

        self._apply_cartesian_control(model, data)
        self.last_input_source = "target_pose"

    def process_gamepad_input(self, raw_control, model, data):
        """Process raw gamepad input into robot controls."""
        axes = raw_control["axes"]
        buttons = raw_control["buttons"]
        
        # Reset position with START button
        if buttons[9]:
            self.reset_pose(model, data)
            return
        # Gripper control with X and Y buttons
        if buttons[0]:  # X button - Close gripper
            data.ctrl[self.gripper_actuator] = self.gripper_range[0]  # Close (0)
            self.gripper_state = 0.0
        elif buttons[3]:  # Y button - Open gripper
            data.ctrl[self.gripper_actuator] = self.gripper_range[1]  # Open (255)
            self.gripper_state = 1.0
        # Speed scaling with bumpers
        if buttons[4]:  # LB
            self.speed_scale = max(0.1, self.speed_scale - 0.1)
        if buttons[5]:  # RB
            self.speed_scale = min(1.0, self.speed_scale + 0.1)
            
        # Get cartesian control inputs with deadzone
        dx = self.apply_deadzone(axes[0], self.deadzone) * self.speed_scale * 0.1
        dy = -self.apply_deadzone(axes[1], self.deadzone) * self.speed_scale * 0.1
        dz = -self.apply_deadzone(axes[3], self.deadzone) * self.speed_scale * 0.1
        
        # Update target position incrementally for gamepad control
        self.target_pos += np.array([dx, dy, dz])
        self._apply_cartesian_control(model, data)
        
        # # Debug output 
        # if np.any(np.abs([dx, dy, dz]) > 0):
        #     print(f"Target: {self.target_pos}")
        #     print(f"Current: {data.body(self.hand_id).xpos}")
        #     print(f"Speed: {self.speed_scale:.1f}")
        #     print(f"Gripper: {'Open' if self.gripper_state > 0.5 else 'Closed'}")
        self.last_input_source = "gamepad"

    def _apply_cartesian_control(self, model, data):
        """Apply cartesian control to move the robot towards target position."""
        twist = np.zeros(6)  # 3 for position, 3 for orientation
        twist[:3] = self.Kpos * (self.target_pos - data.body(self.hand_id).xpos) / self.integration_dt
        
        current_rot_mat = data.body(self.hand_id).xmat.reshape(3, 3)
        desired_rot_mat = self.get_target_rotation_matrix()
        current_rot = transform.Rotation.from_matrix(current_rot_mat)
        desired_rot = transform.Rotation.from_matrix(desired_rot_mat)
        
        rot_error = (desired_rot * current_rot.inv()).as_rotvec()
        twist[3:] = self.Kori * rot_error / self.integration_dt
        
        # Get Jacobian for the hand body
        jacp = np.zeros((3, model.nv))
        jacr = np.zeros((3, model.nv))
        mujoco.mj_jacBody(model, data, jacp, jacr, self.hand_id)
        
        # Extract the relevant part of the Jacobian (just the 7 arm joints)
        jac = np.vstack((jacp[:, :7], jacr[:, :7]))
        
        # Damped least squares
        diag = self.damping * np.eye(6)
        dq = jac.T @ np.linalg.solve(jac @ jac.T + diag, twist)
        
        # Nullspace control
        N = np.eye(7) - np.linalg.pinv(jac) @ jac  # Nullspace projection matrix
        dq_null = self.Kn * (self.home_pos[:7] - data.qpos[:7])  # Joint space velocity
        dq += N @ dq_null  # Add nullspace movement
        
        # Clamp maximum joint velocity
        dq_abs_max = np.abs(dq).max()
        if dq_abs_max > self.max_angvel:
            dq *= self.max_angvel / dq_abs_max
            
        # Integrate joint velocities
        q = data.qpos[:7].copy()  # Only get the 7 arm joints
        q += dq * self.integration_dt
        
        # Apply joint limits
        np.clip(q, model.jnt_range[:7, 0], model.jnt_range[:7, 1], out=q)
        
        # Set control signals for arm joints
        data.ctrl[:7] = q
            
    def apply_deadzone(self, value, deadzone):
        """Apply deadzone to joystick input."""
        if abs(value) < deadzone:
            return 0.0
        return value

def main():
    node = Node("mujoco_franka")
    print("Initializing MuJoCo Franka simulation...")
    
    # Check if all required mesh files are present
    print("Checking mesh files...")
    if not check_mesh_files_exist():
        print("Some mesh files are missing. Downloading required files...")
        if not ensure_meshes():
            print("Error: Failed to download all required mesh files")
            return
    
    # Load the MuJoCo model
    model_path = os.path.join(os.path.dirname(__file__), "franka_emika_panda/scene.xml")
    model = mujoco.MjModel.from_xml_path(model_path)

    
    data = mujoco.MjData(model)
    data.qpos[:7] = np.array([0, 0, 0, -1.57079, 0, 1.57079, -0.7853]) # home position
    mujoco.mj_forward(model, data)
    
    controller = RobotController()
    controller.initialize(model, data)
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("Simulation initialized successfully")
        print("\nGamepad control mapping:")
        print("  Left stick X: Move along X axis")
        print("  Left stick Y: Move along Y axis")
        print("  Right stick Y: Move along Z axis")
        print("  LB/RB: Decrease/Increase speed")
        print("  START: Reset position")
        print("  X: Close gripper")
        print("  Y: Open gripper")

        input_received = False
        
        for event in node:
            if event["type"] == "INPUT":
                input_received = True
                
                if event["id"] == "raw_control":
                    raw_control = json.loads(event["value"].to_pylist()[0])
                    controller.process_gamepad_input(raw_control, model, data)
                    
                elif event["id"] == "target_pose":
                    target_pose = event["value"].to_pylist()
                    # print(f"Received target pose: {target_pose}")
                    controller.process_target_pose(target_pose, model, data)
            
            if input_received:
                if controller.last_input_source == "target_pose":
                    controller._apply_cartesian_control(model, data)
                
                # Step simulation and update viewer
                mujoco.mj_step(model, data)
                viewer.sync()
                
                # Send feedback
                node.send_output(
                    "joint_positions",
                    data=pa.array(data.qpos[:7].tolist(), type=pa.float64()),
                    metadata={"type": "joint_positions"}
                )
                
                node.send_output(
                    "ee_position",
                    data=pa.array(data.body(controller.hand_id).xpos.tolist(), 
                                    type=pa.float64()),
                    metadata={"type": "ee_position"}
                )
            
            else:
                mujoco.mj_step(model, data)
                viewer.sync()


if __name__ == "__main__":
    main()