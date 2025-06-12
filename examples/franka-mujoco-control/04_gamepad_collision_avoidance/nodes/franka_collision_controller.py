"""Enhanced Franka controller with gamepad support and collision avoidance using repulsive forces."""

import json
import time
import numpy as np
import pyarrow as pa
import torch
from dora import Node
import pytorch_kinematics as pk
from scipy.spatial.transform import Rotation

class FrankaCollisionController:
    """Franka controller with gamepad support and self-collision avoidance using repulsive forces."""
    
    def __init__(self):
        """Initialize controller with PyTorch Kinematics and collision avoidance."""
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
        self.Kpos = 0.95  
        self.Kori = 0.95  
        self.Kn = np.array([10.0, 10.0, 10.0, 10.0, 5.0, 5.0, 5.0])  
        self.max_angvel = 0.785
        
        # Collision avoidance parameters
        self.collision_check_enabled = True
        self.min_link_distance = 0.02  
        self.collision_force_gain = 1000.0  
        self.collision_influence_distance = 0.05  
        
        # Define which links can collide with each other (non-adjacent only)
        self.collision_pairs = [
            ('panda_link1', 'panda_link3'), ('panda_link1', 'panda_link4'), ('panda_link1', 'panda_link5'), ('panda_link1', 'panda_link6'), ('panda_link1', 'panda_link7'),
            ('panda_link1', 'panda_hand'), ('panda_link2', 'panda_link4'), ('panda_link2', 'panda_link5'), ('panda_link2', 'panda_link6'), ('panda_link2', 'panda_link7'),
            ('panda_link2', 'panda_hand'), ('panda_link3', 'panda_link5'), ('panda_link3', 'panda_link6'), ('panda_link3', 'panda_link7'), ('panda_link3', 'panda_hand'),
            ('panda_link4', 'panda_link6'), ('panda_link4', 'panda_link7'), ('panda_link4', 'panda_hand'),
            # ('panda_link5', 'panda_hand'), # Enabling this results in very jaggy motion 
        ]
        
        # Approximate link geometries for distance calculation
        self.link_geometries = {
            'panda_link1': {'radius': 0.08, 'length': 0.15, 'offset': np.array([0, 0, 0.075])},
            'panda_link2': {'radius': 0.08, 'length': 0.12, 'offset': np.array([0, 0, 0])},
            'panda_link3': {'radius': 0.07, 'length': 0.32, 'offset': np.array([0, 0, 0.16])},
            'panda_link4': {'radius': 0.07, 'length': 0.28, 'offset': np.array([0, 0, -0.14])},
            'panda_link5': {'radius': 0.06, 'length': 0.22, 'offset': np.array([0, 0, 0.11])},
            'panda_link6': {'radius': 0.06, 'length': 0.12, 'offset': np.array([0, 0, 0.06])},
            'panda_link7': {'radius': 0.05, 'length': 0.08, 'offset': np.array([0, 0, 0.04])},
            'panda_hand': {'radius': 0.07, 'length': 0.07, 'offset': np.array([0, 0, 0.035])},
        }
        
        # Gamepad control parameters
        self.speed_scale = 0.5
        self.deadzone = 0.05
        
        # Gripper control parameters
        self.gripper_range = [0, 255]
        self.gripper_state = 0.0  
        
        # Robot state
        self.current_joint_pos = None
        self.target_pos = np.array([0.55, 0.0, 0.6])  
        self.target_rpy = [180.0, 0.0, 90.0]  
        self.home_pos = np.array([0, 0, 0, -1.57079, 0, 1.57079, -0.7853])
        
        # Collision state tracking
        self.is_in_collision_risk = False
        self.collision_force_applied_count = 0
        self.current_collision_info = ""
        
        # print("Franka Collision Controller initialized")
        # print(f"Chain DOF: {len(self.chain.get_joint_parameter_names())}")
        # print(f"Using device: {self.device}")
    
    def calculate_collision_repulsive_forces(self, joint_angles):
        """Calculate repulsive forces to avoid collisions between robot links."""
        if not self.collision_check_enabled:
            return np.zeros(7), False, ""
        
        # Get forward kinematics for all links
        q_tensor = torch.tensor(joint_angles, device=self.device, dtype=torch.float32).unsqueeze(0)
        fk_results = self.chain.forward_kinematics(q_tensor, end_only=False)
        
        # Initialize total repulsive force in joint space
        total_repulsive_forces = np.zeros(7)
        collision_detected = False
        collision_info_list = []
        
        # Check each collision pair
        for link1, link2 in self.collision_pairs:
            if link1 not in fk_results or link2 not in fk_results:
                continue
            
            # Get link transforms
            transform1 = fk_results[link1].get_matrix().squeeze(0).cpu().numpy()
            transform2 = fk_results[link2].get_matrix().squeeze(0).cpu().numpy()
            
            # Get link center positions
            link1_info, link2_info = self.link_geometries[link1], self.link_geometries[link2]
            offset1, offset2 = link1_info['offset'], link2_info['offset']
            radius1, radius2 = link1_info['radius'], link2_info['radius']

            # Transform offsets to world coordinates
            pos1 = transform1[:3, 3] + transform1[:3, :3] @ offset1
            pos2 = transform2[:3, 3] + transform2[:3, :3] @ offset2
            
            # Calculate distance between link centers
            distance = np.linalg.norm(pos1 - pos2)
            
            # Check if within influence range
            min_safe_distance = radius1 + radius2 + self.min_link_distance
            influence_distance = min_safe_distance + self.collision_influence_distance
            
            if distance < influence_distance and distance > 1e-6:
                collision_detected = True
                
                # Calculate repulsive force magnitude
                if distance < min_safe_distance:
                    force_magnitude = self.collision_force_gain * (1.0/max(distance, 0.01) - 1.0/min_safe_distance)
                else:
                    force_magnitude = 0
                
                # Direction of repulsive force
                if distance > 1e-6:
                    force_direction = (pos1 - pos2) / distance
                else:
                    force_direction = np.array([1, 0, 0])
                
                # Convert Cartesian repulsive force to joint space forces
                link1_forces = self.cartesian_to_joint_forces(link1, pos1, force_direction * force_magnitude, joint_angles)
                link2_forces = self.cartesian_to_joint_forces(link2, pos2, -force_direction * force_magnitude, joint_angles)
                
                total_repulsive_forces += link1_forces + link2_forces
                
                # Store collision info
                collision_info_list.append(f"{link1}<->{link2}: {distance:.2f}m")
        
        collision_info = "; ".join(collision_info_list) if collision_info_list else "No collision"
        return total_repulsive_forces, collision_detected, collision_info
    
    def cartesian_to_joint_forces(self, link_name, link_position, cartesian_force, joint_angles):
        """Convert Cartesian force at a link to joint space forces."""
        # Get numerical Jacobian for this link position
        jacobian = self.get_link_jacobian_simplified(link_name, joint_angles, link_position)
        
        # Convert Cartesian force to joint torques using Jacobian transpose
        joint_forces = jacobian.T @ cartesian_force[:3]  
        
        return joint_forces
    
    def get_link_jacobian_simplified(self, link_name, joint_angles, link_position):
        """Get link Jacobian using numerical differentiation."""
        epsilon = 1e-6
        jacobian = np.zeros((3, 7))
        
        # Calculate numerical Jacobian
        for i in range(7):
            # Perturb joint i
            perturbed_joints = joint_angles.copy()
            perturbed_joints[i] += epsilon
            
            # Get perturbed link position
            q_perturbed = torch.tensor(perturbed_joints, device=self.device, dtype=torch.float32).unsqueeze(0)
            fk_perturbed = self.chain.forward_kinematics(q_perturbed, end_only=False)
            
            if link_name in fk_perturbed:
                transform_perturbed = fk_perturbed[link_name].get_matrix().squeeze(0).cpu().numpy()
                offset = self.link_geometries[link_name]['offset']
                perturbed_pos = transform_perturbed[:3, 3] + transform_perturbed[:3, :3] @ offset
                
                # Numerical derivative
                jacobian[:, i] = (perturbed_pos - link_position) / epsilon
        
        return jacobian
    
    def apply_deadzone(self, value):
        """Apply deadzone to joystick input."""
        return 0.0 if abs(value) < self.deadzone else value
    
    def get_current_ee_pose(self, joint_angles):
        """Get current end-effector pose."""
        q = torch.tensor(joint_angles, device=self.device, dtype=torch.float32).unsqueeze(0)
        tf = self.chain.forward_kinematics(q)
        transform_matrix = tf.get_matrix().squeeze(0).cpu().numpy()
        position = transform_matrix[:3, 3]
        rotation_matrix = transform_matrix[:3, :3]
        return position, rotation_matrix
    
    def compute_jacobian_pytorch(self, joint_angles):
        """Compute Jacobian using PyTorch Kinematics."""
        q = torch.tensor(joint_angles, device=self.device, dtype=torch.float32).unsqueeze(0)
        J = self.chain.jacobian(q)
        return J.squeeze(0).cpu().numpy()
    
    def process_gamepad_input(self, raw_control):
        """Process gamepad input."""
        
        axes = raw_control["axes"]
        buttons = raw_control["buttons"]

        # Reset position with START button
        if len(buttons) > 9 and buttons[9]:
            if self.current_joint_pos is not None:
                home_ee_pos, _ = self.get_current_ee_pose(self.home_pos)
                self.target_pos = home_ee_pos.copy()
                print("Reset target to home position")
        
        # Gripper control with X and Y buttons
        if len(buttons) > 0 and buttons[0]:  
            self.gripper_state = 0.0
            print("Gripper: CLOSED")
        elif len(buttons) > 3 and buttons[3]:  
            self.gripper_state = 1.0
            print("Gripper: OPEN")
        
        # Speed scaling with bumpers (LB/RB)
        if len(buttons) > 4 and buttons[4]:  
            self.speed_scale = max(0.1, self.speed_scale - 0.1)
            print(f"Speed: {self.speed_scale:.1f}")
        if len(buttons) > 5 and buttons[5]:  
            self.speed_scale = min(1.0, self.speed_scale + 0.1)
            print(f"Speed: {self.speed_scale:.1f}")
        
        # Get cartesian control inputs with deadzone 
        dx = self.apply_deadzone(axes[0]) * self.speed_scale * 0.1
        dy = -self.apply_deadzone(axes[1]) * self.speed_scale * 0.1
        dz = -self.apply_deadzone(axes[3]) * self.speed_scale * 0.1
        
        # Update target position incrementally
        if abs(dx) > 0 or abs(dy) > 0 or abs(dz) > 0:
            self.target_pos += np.array([dx, dy, dz])

    def get_target_rotation_matrix(self):
        """Convert RPY to rotation matrix."""
        roll_rad, pitch_rad, yaw_rad = np.radians(self.target_rpy)
        desired_rot = Rotation.from_euler('ZYX', [yaw_rad, pitch_rad, roll_rad])
        return desired_rot.as_matrix()
    
    def apply_cartesian_control(self, current_joints):
        """Apply Cartesian control with collision avoidance using repulsive forces."""
        self.current_joint_pos = current_joints[:7]
        
        # Get current end-effector pose
        current_ee_pos, current_ee_rot = self.get_current_ee_pose(self.current_joint_pos)
        
        # Calculate position error
        pos_error = self.target_pos - current_ee_pos

        # Construct 6D twist
        twist = np.zeros(6)
        twist[:3] = self.Kpos * pos_error / self.integration_dt
        
        # Orientation control
        current_rot = Rotation.from_matrix(current_ee_rot)
        desired_rot = Rotation.from_matrix(self.get_target_rotation_matrix())
        rot_error = (desired_rot * current_rot.inv()).as_rotvec()
        twist[3:] = self.Kori * rot_error / self.integration_dt
        
        # Get Jacobian
        jac = self.compute_jacobian_pytorch(self.current_joint_pos)  
        
        # Damped least squares inverse kinematics
        diag = self.damping * np.eye(6)
        dq = jac.T @ np.linalg.solve(jac @ jac.T + diag, twist)
        
        # Nullspace control
        jac_pinv = np.linalg.pinv(jac)
        N = np.eye(7) - jac_pinv @ jac  
        dq_null = self.Kn * (self.home_pos - self.current_joint_pos)  
        dq += N @ dq_null  
        
        # Collision avoidance
        collision_forces, collision_detected, collision_info = self.calculate_collision_repulsive_forces(self.current_joint_pos)
        
        if collision_detected:
            dq += collision_forces * self.integration_dt
            self.is_in_collision_risk = True
            self.current_collision_info = collision_info
            # print(f"Active collisions: {collision_info}")
        else:
            if self.is_in_collision_risk:
                print("Collision forces cleared - normal motion resumed")
            self.is_in_collision_risk = False
            self.current_collision_info = "Safe"
        
        # Limit joint velocities
        dq_abs_max = np.abs(dq).max()
        if dq_abs_max > self.max_angvel:
            dq *= self.max_angvel / dq_abs_max
            
        # Integrate to get new joint positions
        new_joints = self.current_joint_pos + dq * self.integration_dt
        
        # Apply joint limits
        joint_limits = np.array([[-2.8973, 2.8973], [-1.7628, 1.7628], [-2.8973, 2.8973], [-3.0718, -0.0698], [-2.8973, 2.8973], [-0.0175, 3.7525], [-2.8973, 2.8973]])
        
        for i in range(7):
            new_joints[i] = np.clip(new_joints[i], joint_limits[i][0], joint_limits[i][1])
        
        # Return commands
        full_commands = np.zeros(8)
        full_commands[:7] = new_joints
        full_commands[7] = self.gripper_range[0] if self.gripper_state < 0.5 else self.gripper_range[1]
        
        return full_commands


def main():
    """Main controller loop."""
    node = Node("franka_collision_controller")
    controller = FrankaCollisionController()
    
    print("Franka Collision Controller Node Started")
    
    for event in node:
        if event["type"] == "INPUT":
            
            if event["id"] == "joint_positions":
                joint_positions = event["value"].to_numpy()
                full_commands = controller.apply_cartesian_control(joint_positions)
                
                node.send_output(
                    "joint_commands",
                    pa.array(full_commands, type=pa.float64()),
                    metadata={"timestamp": time.time()}
                )
                
                if controller.current_joint_pos is not None:
                    current_ee_pos, _ = controller.get_current_ee_pose(controller.current_joint_pos)
                    node.send_output(
                        "ee_position",
                        pa.array(current_ee_pos, type=pa.float64()),
                        metadata={"timestamp": time.time()}
                    )
                
                # Send collision status
                collision_status = {
                    "collision_risk": controller.is_in_collision_risk,
                    "collision_info": controller.current_collision_info,
                    "force_applied_count": controller.collision_force_applied_count
                }
                node.send_output(
                    "collision_status",
                    pa.array([1.0 if controller.is_in_collision_risk else 0.0], type=pa.float64()),
                    metadata={"collision_status": json.dumps(collision_status)}
                )
                
            elif event["id"] == "raw_control":
                raw_control = json.loads(event["value"].to_pylist()[0])
                controller.process_gamepad_input(raw_control)
                

if __name__ == "__main__":
    main()