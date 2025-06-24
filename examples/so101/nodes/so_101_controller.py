import json
import time
import numpy as np
import pyarrow as pa
from dora import Node
from scipy.spatial.transform import Rotation

class GamepadController:
    def __init__(self):
        # Control parameters
        self.integration_dt = 0.1
        self.damping = 1e-4
        self.Kpos = 0.95  # Position gain
        self.Kori = 0.95  # Orientation gain

        # Gamepad control parameters
        self.speed_scale = 0.5
        self.max_angvel = 1.57 * self.speed_scale  # Max joint velocity (rad/s)
        
        # Robot state variables
        self.dof = None
        self.current_joint_pos = None  # Full robot state
        self.home_pos = None  # Home position for arm joints only
        
        # Target pose (independent of DOF)
        self.target_pos = np.array([0.0, -0.2, 0.15])  # Conservative default target
        self.target_rpy = [0.0, 0.0, 0.0]  # Downward orientation
        
        # Gripper control (0° = closed, 95° = fully open)
        self.gripper_position = 0.0  # Current gripper position in degrees
        self.gripper_rate = 5.0  
        
        # Cache for external computations
        self.current_ee_pose = None  # End-effector pose
        self.current_jacobian = None  # Jacobian matrix
        
        print("Gamepad Controller initialized")
    
    def _initialize_robot(self, positions, jacobian_dof=None):
        self.full_joint_count = len(positions)
        self.dof = jacobian_dof if jacobian_dof is not None else self.full_joint_count
        self.current_joint_pos = positions.copy()
        self.home_pos = np.zeros(self.dof)
    
    def process_cmd_vel(self, cmd_vel):
        # print(f"Processing cmd_vel: {cmd_vel}")
        
        # Position control (first 3 elements)
        delta_pos = cmd_vel[:3] * 0.03 * self.speed_scale
        dy, dx, dz = delta_pos
        
        # Update target position incrementally
        if abs(dx) > 0 or abs(dy) > 0 or abs(dz) > 0:
            self.target_pos += np.array([-dx, -dy, dz])
        
        # Orientation control (last 3 elements) - convert to degrees
        delta_rpy = cmd_vel[3:6] * 5.0 * self.speed_scale  # Scale factor for rotation (degrees)
        droll, dpitch, dyaw = delta_rpy
        
        # Update target orientation incrementally (in degrees)
        self.target_rpy[0] += droll   # Roll
        self.target_rpy[1] += dpitch  # Pitch  
        self.target_rpy[2] += dyaw    # Yaw
        
        # Keep angles in reasonable range (-180 to 180 degrees)
        self.target_rpy = [(angle + 180) % 360 - 180 for angle in self.target_rpy]
        
        # print(f"Updated target position: {self.target_pos}, target orientation (RPY degrees): {self.target_rpy}")
    
    def process_gamepad_input(self, raw_control):
        buttons = raw_control["buttons"]

        # Speed scaling with bumpers (LB/RB)
        if len(buttons) > 4 and buttons[4]:  # LB
            self.speed_scale = max(0.1, self.speed_scale - 0.1)
            print(f"Speed: {self.speed_scale:.1f}")
        if len(buttons) > 5 and buttons[5]:  # RB
            self.speed_scale = min(1.0, self.speed_scale + 0.1)
            print(f"Speed: {self.speed_scale:.1f}")
        
        # Button A  - Open gripper (increase angle)
        if buttons[1]:
            self.gripper_position += self.gripper_rate * self.integration_dt
            
        # Button B - Close gripper (decrease angle)
        if buttons[2]:
            self.gripper_position -= self.gripper_rate * self.integration_dt
            
        # Clamp gripper position to valid range (0° = closed, 95° = fully open)
        self.gripper_position = np.clip(self.gripper_position, 0.0, 95.0)

    def get_target_rotation_matrix(self):
        roll_rad, pitch_rad, yaw_rad = np.radians(self.target_rpy)
        desired_rot = Rotation.from_euler('ZYX', [yaw_rad, pitch_rad, roll_rad])
        return desired_rot.as_matrix()
    
    def update_jacobian(self, jacobian_flat, shape):
        jacobian_dof = shape[1]
        self.current_jacobian = jacobian_flat.reshape(shape)

        if self.dof is None:
            if self.current_joint_pos is not None:
                self._initialize_robot(self.current_joint_pos, jacobian_dof)
            else:
                self.dof = jacobian_dof
        elif self.dof != jacobian_dof:
            self.dof = jacobian_dof
            self.home_pos = np.zeros(self.dof)
    
    def apply_differential_ik_control(self):
        # Return current position if no data available
        if self.current_ee_pose is None or self.current_jacobian is None:
            if self.current_joint_pos is not None:
                # Update gripper position
                new_joints = self.current_joint_pos.copy()
                if len(new_joints) > self.dof:
                    new_joints[self.dof] = np.radians(self.gripper_position)
                return new_joints
            return self.current_joint_pos
            
        current_ee_pos = self.current_ee_pose['position']
        current_ee_rpy = self.current_ee_pose['rpy']

        # print(f"Current EE Position: {current_ee_pos}, RPY: {current_ee_rpy}")
        # print(f"Updated target position: {self.target_pos}, target orientation (RPY degrees): {self.target_rpy}")
        pos_error = self.target_pos - current_ee_pos
        twist = np.zeros(6)
        twist[:3] = self.Kpos * pos_error / self.integration_dt
        
        # Convert current RPY to rotation matrix
        current_rot = Rotation.from_euler('XYZ', current_ee_rpy)
        desired_rot = Rotation.from_matrix(self.get_target_rotation_matrix())
        rot_error = (desired_rot * current_rot.inv()).as_rotvec()
        twist[3:] = self.Kori * rot_error / self.integration_dt
        
        jac = self.current_jacobian
        
        # Damped least squares inverse kinematics
        diag = self.damping * np.eye(6)
        dq = jac.T @ np.linalg.solve(jac @ jac.T + diag, twist)
        
        # # Apply nullspace control
        current_arm = self.current_joint_pos[:self.dof]
        jac_pinv = np.linalg.pinv(jac)
        N = np.eye(self.dof) - jac_pinv @ jac
        k_null = np.ones(self.dof) * 5.0
        dq_null = k_null * (self.home_pos - current_arm)
        dq += N @ dq_null
        
        # Limit joint velocities
        dq_abs_max = np.abs(dq).max()
        if dq_abs_max > self.max_angvel:
            dq *= self.max_angvel / dq_abs_max
            
        # Create full joint command
        new_joints = self.current_joint_pos.copy()
        new_joints[:self.dof] += dq * self.integration_dt
        new_joints[self.dof] = np.radians(self.gripper_position)
        
        return new_joints


def main():
    node = Node()
    controller = GamepadController()
    
    for event in node:
        if event["type"] == "INPUT":
            
            if event["id"] == "joint_positions":
                # dora-rustypot sends joint positions as Vec<f64>
                joint_pos = event["value"].to_numpy().astype(np.float64)
                
                if controller.current_joint_pos is None:
                    controller._initialize_robot(joint_pos)
                else:
                    controller.current_joint_pos = joint_pos
                
                # Request FK computation
                node.send_output(
                    "fk_request", 
                    pa.array(controller.current_joint_pos, type=pa.float32()),
                    metadata={"encoding": "jointstate", "timestamp": time.time()}
                )
                
                # Request Jacobian computation
                node.send_output(
                    "jacobian_request", 
                    pa.array(controller.current_joint_pos, type=pa.float32()),
                    metadata={"encoding": "jacobian", "timestamp": time.time()}
                )
                
                # Apply differential IK control
                joint_commands = controller.apply_differential_ik_control()
                # Send control commands to the robot
                node.send_output(
                    "joint_commands", 
                    pa.array(joint_commands.astype(np.float64), type=pa.float64()),
                    metadata={"timestamp": time.time()}
                )
                
            elif event["id"] == "raw_control":
                raw_control = json.loads(event["value"].to_pylist()[0])
                controller.process_gamepad_input(raw_control)
                
            elif event["id"] == "cmd_vel":
                cmd_vel = event["value"].to_numpy()
                controller.process_cmd_vel(cmd_vel)
            
            # Handle FK results
            if event["id"] == "fk_result":
                if event["metadata"].get("encoding") == "xyzrpy":
                    ee_pose = event["value"].to_numpy()
                    controller.current_ee_pose = {'position': ee_pose[:3], 'rpy': ee_pose[3:6]}
                    
            # Handle Jacobian results
            if event["id"] == "jacobian_result":
                if event["metadata"].get("encoding") == "jacobian_result":
                    jacobian_flat = event["value"].to_numpy()
                    jacobian_shape = event["metadata"]["jacobian_shape"]
                    controller.update_jacobian(jacobian_flat, jacobian_shape)

if __name__ == "__main__":
    main()