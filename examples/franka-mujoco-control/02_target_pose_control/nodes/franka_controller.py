"""Franka robot controller node for Dora.
"""

import time
import numpy as np
import pyarrow as pa
import mujoco
from dora import Node
from scipy.spatial import transform
from robot_descriptions.loaders.mujoco import load_robot_description

class FrankaController:
    """Franka Panda robot controller using proven MuJoCo-based control."""
    
    def __init__(self):
        """
        Initialize the Franka robot controller with MuJoCo simulation.
        Sets up the robot model, simulation data, and control parameters for end-effector
        pose control using operational space control with nullspace projection.
        """

        self.model = load_robot_description("panda_mj_description", variant="scene")
        self.data = mujoco.MjData(self.model)
        
        # Get the hand body ID for end-effector control
        self.hand_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "hand")
        
        # Control parameters
        self.integration_dt = 0.1
        self.damping = 1e-4
        self.Kpos = 0.95  # Position gain
        self.Kori = 0.95  # Orientation gain
        self.Kn = np.array([10.0, 10.0, 10.0, 10.0, 5.0, 5.0, 5.0])  # Nullspace gains
        self.max_angvel = 0.785  # Max joint velocity (rad/s)
        
        # Robot state
        self.current_joint_pos = None
        self.target_pos = np.array([0.4, 0.0, 0.3])  # Conservative default target
        self.target_rpy = [180.0, 0.0, 90.0]  # Downward orientation
        self.home_pos = np.array([0, 0, 0, -1.57079, 0, 1.57079, -0.7853])
        
        # Initialize simulation data with home position
        self.data.qpos[:7] = self.home_pos
        mujoco.mj_forward(self.model, self.data)
        
        # Initialize target position to current end-effector position
        self.target_pos = self.data.body(self.hand_id).xpos.copy()
        print("Franka Controller initialized with MuJoCo model")
    
    def set_target_pose(self, pose_array):
        """Set target pose from input array."""
        if len(pose_array) >= 3:
            self.target_pos = np.array(pose_array[:3])
        
        if len(pose_array) >= 6:
            self.target_rpy = list(pose_array[3:6])
    
    def get_target_rotation_matrix(self):
        """Convert RPY to rotation matrix."""
        roll_rad, pitch_rad, yaw_rad = np.radians(self.target_rpy)
        desired_rot = transform.Rotation.from_euler('ZYX', [yaw_rad, pitch_rad, roll_rad])
        return desired_rot.as_matrix()
    
    def apply_cartesian_control(self, current_joints):
        """Apply Cartesian control using the exact same method as dora-franka-mujoco."""
        # Update our internal model state with current joint positions
        self.data.qpos[:7] = current_joints[:7]
        mujoco.mj_forward(self.model, self.data)
        
        # Get current end-effector pose
        current_ee_pos = self.data.body(self.hand_id).xpos.copy()
        current_ee_rot = self.data.body(self.hand_id).xmat.reshape(3, 3)
        
        pos_error = self.target_pos - current_ee_pos
        
        # Construct 6D twist (3 position + 3 orientation)
        twist = np.zeros(6)
        twist[:3] = self.Kpos * pos_error / self.integration_dt
        
        # Orientation control
        current_rot = transform.Rotation.from_matrix(current_ee_rot)
        desired_rot = transform.Rotation.from_matrix(self.get_target_rotation_matrix())
        rot_error = (desired_rot * current_rot.inv()).as_rotvec()
        twist[3:] = self.Kori * rot_error / self.integration_dt
        
        # Get Jacobian for the hand body
        jacp = np.zeros((3, self.model.nv))  # Position Jacobian
        jacr = np.zeros((3, self.model.nv))  # Rotation Jacobian
        mujoco.mj_jacBody(self.model, self.data, jacp, jacr, self.hand_id)
        
        # Extract only the arm joints (first 7 DOF)
        jac = np.vstack((jacp[:, :7], jacr[:, :7]))
        
        # Damped least squares inverse kinematics
        diag = self.damping * np.eye(6)
        dq = jac.T @ np.linalg.solve(jac @ jac.T + diag, twist)
        
        # Nullspace control - drive towards home position in nullspace
        N = np.eye(7) - np.linalg.pinv(jac) @ jac  # Nullspace projection
        dq_null = self.Kn * (self.home_pos - current_joints[:7])  # Nullspace velocity
        dq += N @ dq_null  # Add nullspace movement
        
        # Limit joint velocities
        dq_abs_max = np.abs(dq).max()
        if dq_abs_max > self.max_angvel:
            dq *= self.max_angvel / dq_abs_max
            
        # Integrate to get new joint positions
        new_joints = current_joints[:7] + dq * self.integration_dt
        
        # Apply joint limits (from MuJoCo model)
        for i in range(7):
            joint_range = self.model.jnt_range[i]
            new_joints[i] = np.clip(new_joints[i], joint_range[0], joint_range[1])
        
        return new_joints

def main():
    node = Node("franka_controller")
    controller = FrankaController()
    
    for event in node:
        if event["type"] == "INPUT":
            
            if event["id"] == "joint_positions":
                # Update current state and compute control commands
                controller.current_joint_pos = event["value"].to_numpy()
                
                # Apply Cartesian control using proven method
                joint_commands = controller.apply_cartesian_control(controller.current_joint_pos)
                
                # Send joint commands
                node.send_output(
                    "joint_commands",
                    pa.array(joint_commands, type=pa.float64()),
                    metadata={"timestamp": time.time()}
                )
                
                # Send current end-effector position
                if controller.hand_id is not None:
                    ee_pos = controller.data.body(controller.hand_id).xpos.copy()
                    node.send_output(
                        "ee_position", 
                        pa.array(ee_pos, type=pa.float64()),
                        metadata={"timestamp": time.time()}
                    )
                
            elif event["id"] == "target_pose":
                # Process new target pose
                pose_array = event["value"].to_numpy()
                controller.set_target_pose(pose_array)

if __name__ == "__main__":
    main()