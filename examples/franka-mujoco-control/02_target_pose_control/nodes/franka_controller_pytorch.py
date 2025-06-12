import torch
import numpy as np
import time
import pyarrow as pa
from dora import Node
import pytorch_kinematics as pk
from scipy.spatial.transform import Rotation

class FrankaController:
    def __init__(self):
        """
        Initialize the Franka controller with differential IK nullspace control
        """
        # Load the robot model for IK and FK
        urdf_path = "../franka_panda/panda.urdf"
        with open(urdf_path, 'rb') as f:
            urdf_content = f.read()
        
        # Build serial chain for kinematics
        self.chain = pk.build_serial_chain_from_urdf(urdf_content, "panda_hand")
        
        # Move to GPU if available
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.chain = self.chain.to(device=self.device)
        
        # Control parameters
        self.integration_dt = 0.1
        self.damping = 1e-4
        self.Kpos = 0.95  # Position gain
        self.Kori = 0.95  # Orientation gain
        self.Kn = np.array([10.0, 10.0, 10.0, 10.0, 5.0, 5.0, 5.0])  # Nullspace gains
        self.max_angvel = 0.785  # Max joint velocity (rad/s)
        
        # State variables
        self.current_joint_pos = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
        self.current_joint_vel = np.zeros(7)
        self.target_pos = np.array([0.4, 0.0, 0.3])  # Conservative default target
        self.target_rpy = [180.0, 0.0, 90.0]  # Downward orientation
        self.home_pos = np.array([0, 0, 0, -1.57079, 0, 1.57079, -0.7853]) # joint angles for home position
        
        # Initialize timestamps
        self.last_time = time.time()
        
        print("FrankaController initialized with PyTorch Kinematics Differential IK")
        # print(f"Chain DOF: {len(self.chain.get_joint_parameter_names())}")
        
    def get_target_rotation_matrix(self):
        """Convert RPY to rotation matrix."""
        roll_rad, pitch_rad, yaw_rad = np.radians(self.target_rpy)
        desired_rot = Rotation.from_euler('ZYX', [yaw_rad, pitch_rad, roll_rad])
        return desired_rot.as_matrix()
    
    def compute_jacobian_pytorch(self, joint_angles):
        """Compute Jacobian using PyTorch Kinematics."""
        # Convert to torch tensor
        q = torch.tensor(joint_angles, device=self.device, dtype=torch.float32).unsqueeze(0)
        
        # Compute Jacobian (returns 6x7 matrix: [linear_vel; angular_vel])
        J = self.chain.jacobian(q)  # Shape: (1, 6, 7)
        
        return J.squeeze(0).cpu().numpy()  # Convert back to numpy (6, 7)
    
    def get_current_ee_pose(self, joint_angles):
        """Get current end-effector pose using forward kinematics."""
        # Convert to torch tensor
        q = torch.tensor(joint_angles, device=self.device, dtype=torch.float32).unsqueeze(0)
        
        # Forward kinematics
        tf = self.chain.forward_kinematics(q)  # Returns Transform3d
        
        # Extract position and rotation
        transform_matrix = tf.get_matrix().squeeze(0).cpu().numpy()  # (4, 4)
        position = transform_matrix[:3, 3]
        rotation_matrix = transform_matrix[:3, :3]
        
        return position, rotation_matrix
    
    def update_joint_state(self, positions, velocities):
        """Update the current joint state."""
        # Filter to only use first 7 joints 
        self.current_joint_pos = positions[:7]
        self.current_joint_vel = velocities[:7]
    
    def set_target_pose(self, pose_array):
        """Set target pose from input array."""
        if len(pose_array) >= 3:
            self.target_pos = np.array(pose_array[:3])
            # print(f"Updated target position: {self.target_pos}")
        
        if len(pose_array) >= 6:
            self.target_rpy = list(pose_array[3:6])
            # print(f"Updated target orientation (RPY): {self.target_rpy}")
    
    def apply_differential_ik_control(self):
        """Apply differential IK control with nullspace projection."""
        current_ee_pos, current_ee_rot = self.get_current_ee_pose(self.current_joint_pos)

        pos_error = self.target_pos - current_ee_pos
        twist = np.zeros(6)
        twist[:3] = self.Kpos * pos_error / self.integration_dt
        
        current_rot = Rotation.from_matrix(current_ee_rot)
        desired_rot = Rotation.from_matrix(self.get_target_rotation_matrix())
        rot_error = (desired_rot * current_rot.inv()).as_rotvec()
        twist[3:] = self.Kori * rot_error / self.integration_dt
        
        # Get Jacobian using PyTorch Kinematics
        jac = self.compute_jacobian_pytorch(self.current_joint_pos)  # (6, 7)
        
        # Damped least squares inverse kinematics
        diag = self.damping * np.eye(6)
        dq = jac.T @ np.linalg.solve(jac @ jac.T + diag, twist)
        
        # Nullspace control
        jac_pinv = np.linalg.pinv(jac)
        N = np.eye(7) - jac_pinv @ jac  # Nullspace projection matrix
        dq_null = self.Kn * (self.home_pos - self.current_joint_pos)  # Nullspace velocity
        dq += N @ dq_null  # nullspace movement
        
        # Limit joint velocities
        dq_abs_max = np.abs(dq).max()
        if dq_abs_max > self.max_angvel:
            dq *= self.max_angvel / dq_abs_max
            
        # Integrate to get new joint positions
        new_joints = self.current_joint_pos + dq * self.integration_dt
        
        # Apply joint limits (simple clipping - you could load from URDF)
        joint_limits = np.array([
            [-2.8973, 2.8973],
            [-1.7628, 1.7628], 
            [-2.8973, 2.8973],
            [-3.0718, -0.0698],
            [-2.8973, 2.8973],
            [-0.0175, 3.7525],
            [-2.8973, 2.8973]
        ])
        
        for i in range(7):
            new_joints[i] = np.clip(new_joints[i], joint_limits[i][0], joint_limits[i][1])
        
        # Debug output
        # pos_error_mag = np.linalg.norm(pos_error)
        # rot_error_mag = np.linalg.norm(rot_error)
        # print(f"Position error: {pos_error_mag:.4f}, Rotation error: {rot_error_mag:.4f}")
        # print(f"Joint velocity magnitude: {np.linalg.norm(dq):.4f}")
        
        return new_joints

def main():
    node = Node()
    
    controller = FrankaController()
    
    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "joint_positions":
                joint_pos = event["value"].to_numpy()
                controller.update_joint_state(joint_pos, controller.current_joint_vel)
                # Apply differential IK control
                joint_commands = controller.apply_differential_ik_control()
                
                # Pad control to match full robot DOF if needed
                if len(joint_commands) == 7:  # Arm only
                    # Add zero control for gripper joints
                    full_control = np.zeros(9)
                    full_control[:7] = joint_commands
                    control_to_send = full_control
                else:
                    control_to_send = joint_commands
                
                # Send control commands to the robot
                node.send_output(
                    "joint_commands", 
                    pa.array(control_to_send, type=pa.float32()),
                    metadata={"timestamp": time.time()}
                )
                
                # Send current end-effector position 
                current_ee_pos, _ = controller.get_current_ee_pose(controller.current_joint_pos)
                node.send_output(
                    "ee_position", 
                    pa.array(current_ee_pos, type=pa.float32()),
                    metadata={"timestamp": time.time()}
                )
            
            if event["id"] == "joint_velocities":
                joint_vel = event["value"].to_numpy()
                controller.update_joint_state(controller.current_joint_pos, joint_vel)
            
            # Handle target pose updates
            if event["id"] == "target_pose":
                target_pose = event["value"].to_numpy()
                # print(f"Received target pose from publisher: {target_pose}")
                controller.set_target_pose(target_pose)

if __name__ == "__main__":
    main()