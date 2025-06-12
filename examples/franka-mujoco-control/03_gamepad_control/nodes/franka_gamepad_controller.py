"""Enhanced Franka controller with gamepad support using PyTorch Kinematics.
"""

import json
import time
import numpy as np
import pyarrow as pa
import torch
from dora import Node
import pytorch_kinematics as pk
from scipy.spatial.transform import Rotation

class EnhancedFrankaController:
    """Franka controller with gamepad and target pose support using PyTorch Kinematics."""
    
    def __init__(self):
        """Initialize controller with PyTorch Kinematics."""
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
        
        # Gamepad control parameters
        self.speed_scale = 0.5
        self.deadzone = 0.05
        self.last_input_source = None
        
        # Gripper control parameters
        self.gripper_range = [0, 255]
        self.gripper_state = 0.0  # (0=closed, 1=open)
        
        # Robot state
        self.current_joint_pos = None
        self.target_pos = np.array([0.55, 0.0, 0.6])  # Conservative default target
        self.target_rpy = [180.0, 0.0, 90.0]  # Downward orientation
        self.home_pos = np.array([0, 0, 0, -1.67079, 0, 1.64079, -0.7853])
        
        print("Enhanced Franka Controller initialized with PyTorch Kinematics")
        # print(f"Chain DOF: {len(self.chain.get_joint_parameter_names())}")
    
    def apply_deadzone(self, value, deadzone=None):
        """Apply deadzone to joystick input."""
        deadzone = deadzone or self.deadzone
        return 0.0 if abs(value) < deadzone else value
    
    def get_current_ee_pose(self, joint_angles):
        """Get current end-effector pose using PyTorch Kinematics forward kinematics."""
        q = torch.tensor(joint_angles, device=self.device, dtype=torch.float32).unsqueeze(0)
        # Forward kinematics
        tf = self.chain.forward_kinematics(q)  # Returns Transform3d
        # Extract position and rotation
        transform_matrix = tf.get_matrix().squeeze(0).cpu().numpy()  # (4, 4)
        position = transform_matrix[:3, 3]
        rotation_matrix = transform_matrix[:3, :3]
        
        return position, rotation_matrix
    
    def compute_jacobian_pytorch(self, joint_angles):
        """Compute Jacobian using PyTorch Kinematics."""
        q = torch.tensor(joint_angles, device=self.device, dtype=torch.float32).unsqueeze(0)
        # Compute Jacobian (returns 6x7 matrix: [linear_vel; angular_vel])
        J = self.chain.jacobian(q)  # Shape: (1, 6, 7)
        
        return J.squeeze(0).cpu().numpy()  # Convert back to numpy (6, 7)
    
    def process_gamepad_input(self, raw_control):
        """Process gamepad input exactly like the original."""
        axes = raw_control["axes"]
        buttons = raw_control["buttons"]

        # Reset position with START button
        if len(buttons) > 9 and buttons[9]:
            # Reset target to a position based on home joint angles
            if self.current_joint_pos is not None:
                # Use current joint positions to get home EE position
                home_ee_pos, _ = self.get_current_ee_pose(self.home_pos)
                self.target_pos = home_ee_pos.copy()
                print("Reset target to home position")
        
        # Gripper control with X and Y buttons (exactly like original)
        if len(buttons) > 0 and buttons[0]:  # X button - Close gripper
            self.gripper_state = 0.0
            print("Gripper: CLOSED")
        elif len(buttons) > 3 and buttons[3]:  # Y button - Open gripper
            self.gripper_state = 1.0
            print("Gripper: OPEN")
        
        # Speed scaling with bumpers (LB/RB)
        if len(buttons) > 4 and buttons[4]:  # LB
            self.speed_scale = max(0.1, self.speed_scale - 0.1)
            print(f"Speed: {self.speed_scale:.1f}")
        if len(buttons) > 5 and buttons[5]:  # RB
            self.speed_scale = min(1.0, self.speed_scale + 0.1)
            print(f"Speed: {self.speed_scale:.1f}")
        
        # Get cartesian control inputs with deadzone 
        dx = self.apply_deadzone(axes[0], self.deadzone) * self.speed_scale * 0.1
        dy = -self.apply_deadzone(axes[1], self.deadzone) * self.speed_scale * 0.1
        dz = -self.apply_deadzone(axes[3], self.deadzone) * self.speed_scale * 0.1
        
        # Update target position incrementally
        if abs(dx) > 0 or abs(dy) > 0 or abs(dz) > 0:
            self.target_pos += np.array([dx, dy, dz])
            self.last_input_source = "gamepad"
            
            # print(f"Gamepad control: dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f}")
            # print(f"New target: {self.target_pos}")

    def process_target_pose(self, pose_array):
        """Process target pose command."""
        if len(pose_array) >= 3:
            self.target_pos = np.array(pose_array[:3])
            print(f"Updated target position: {self.target_pos}")
        
        if len(pose_array) >= 6:
            self.target_rpy = list(pose_array[3:6])
            print(f"Updated target orientation (RPY): {self.target_rpy}")
        
        self.last_input_source = "target_pose"
    
    def get_target_rotation_matrix(self):
        """Convert RPY to rotation matrix."""
        roll_rad, pitch_rad, yaw_rad = np.radians(self.target_rpy)
        desired_rot = Rotation.from_euler('ZYX', [yaw_rad, pitch_rad, roll_rad])
        return desired_rot.as_matrix()
    
    def apply_cartesian_control(self, current_joints):
        """Apply Cartesian control using PyTorch Kinematics differential IK."""
        self.current_joint_pos = current_joints[:7]
        # Get current end-effector pose using PyTorch Kinematics FK
        current_ee_pos, current_ee_rot = self.get_current_ee_pose(self.current_joint_pos)
        
        # Calculate position error
        pos_error = self.target_pos - current_ee_pos

        # Construct 6D twist (3 position + 3 orientation)
        twist = np.zeros(6)
        twist[:3] = self.Kpos * pos_error / self.integration_dt
        
        # Orientation control
        current_rot = Rotation.from_matrix(current_ee_rot)
        desired_rot = Rotation.from_matrix(self.get_target_rotation_matrix())
        rot_error = (desired_rot * current_rot.inv()).as_rotvec()
        twist[3:] = self.Kori * rot_error / self.integration_dt
        
        jac = self.compute_jacobian_pytorch(self.current_joint_pos)  # (6, 7)
        # Damped least squares inverse kinematics
        diag = self.damping * np.eye(6)
        dq = jac.T @ np.linalg.solve(jac @ jac.T + diag, twist)
        
        # Nullspace control - drive towards home position in nullspace
        jac_pinv = np.linalg.pinv(jac)
        N = np.eye(7) - jac_pinv @ jac  # Nullspace projection matrix
        dq_null = self.Kn * (self.home_pos - self.current_joint_pos)  # Nullspace velocity
        dq += N @ dq_null  # Add nullspace movement
        
        # Limit joint velocities
        dq_abs_max = np.abs(dq).max()
        if dq_abs_max > self.max_angvel:
            dq *= self.max_angvel / dq_abs_max
            
        # Integrate to get new joint positions
        new_joints = self.current_joint_pos + dq * self.integration_dt
        
        # Apply joint limits (Franka Panda limits)
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
        
        # Return 8-dimensional control: 7 arm joints + gripper
        full_commands = np.zeros(8)
        full_commands[:7] = new_joints
        # Map gripper state to control range
        full_commands[7] = self.gripper_range[0] if self.gripper_state < 0.5 else self.gripper_range[1]
        
        # Debug output
        # pos_error_mag = np.linalg.norm(pos_error)
        # if pos_error_mag > 0.01:  # Only print if there's significant error
        #     print(f"Position error: {pos_error_mag:.4f}")
    
        return full_commands


def main():
    node = Node("franka_controller")
    controller = EnhancedFrankaController()
    
    print("Enhanced Franka Controller Node Started")
    print("\nGamepad Controls:")
    print("  Left Stick X/Y: Move in X/Y plane")
    print("  Right Stick Y: Move up/down (Z)")
    print("  LB/RB: Decrease/Increase speed")
    print("  START: Reset to home position")
    print("  X: Close gripper")
    print("  Y: Open gripper")
    print("\nAlso accepts target_pose commands")
    print("Ready to receive inputs...")
    
    for event in node:
        if event["type"] == "INPUT":
            
            if event["id"] == "joint_positions":
                # Update current state and compute commands
                joint_positions = event["value"].to_numpy()
                full_commands = controller.apply_cartesian_control(joint_positions)
                
                node.send_output(
                    "joint_commands",
                    pa.array(full_commands, type=pa.float64()),
                    metadata={"timestamp": time.time()}
                )
                
                # Send current end-effector position
                if controller.current_joint_pos is not None:
                    current_ee_pos, _ = controller.get_current_ee_pose(controller.current_joint_pos)
                    node.send_output(
                        "ee_position",
                        pa.array(current_ee_pos, type=pa.float64()),
                        metadata={"timestamp": time.time()}
                    )
                
            elif event["id"] == "raw_control":
                raw_control = json.loads(event["value"].to_pylist()[0])
                controller.process_gamepad_input(raw_control)
                
            elif event["id"] == "target_pose":
                pose_array = event["value"].to_numpy()
                controller.process_target_pose(pose_array)

if __name__ == "__main__":
    main()