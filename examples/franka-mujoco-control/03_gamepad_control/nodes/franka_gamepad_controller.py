"""Enhanced Franka controller with gamepad support.
"""

import json
import time
import numpy as np
import pyarrow as pa
import mujoco
from dora import Node
from scipy.spatial import transform
from robot_descriptions.loaders.mujoco import load_robot_description

class EnhancedFrankaController:
    """Franka controller with gamepad and target pose support using proven MuJoCo control."""
    
    def __init__(self):
        """Init"""
        self.model = load_robot_description("panda_mj_description", variant="scene")
        self.data = mujoco.MjData(self.model)
        
        # Get the hand body ID for end-effector control
        self.hand_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "hand")
        # Get gripper actuator ID (like in original)
        self.gripper_actuator = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "actuator8")
        
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
        self.target_pos = np.array([0.4, 0.0, 0.3])  # Conservative default target
        self.target_rpy = [180.0, 0.0, 90.0]  # Downward orientation
        self.home_pos = np.array([0, 0, 0, -1.57079, 0, 1.57079, -0.7853])
        
        # Initialize simulation data with home position
        self.data.qpos[:7] = self.home_pos
        mujoco.mj_forward(self.model, self.data)
        
        # Initialize target position to current end-effector position
        self.target_pos = self.data.body(self.hand_id).xpos.copy()
        
        print("Enhanced Franka Controller initialized with MuJoCo model")
        # print(f"Hand body ID: {self.hand_id}")
        # print(f"Gripper actuator ID: {self.gripper_actuator}")
    
    def apply_deadzone(self, value, deadzone=None):
        """Apply deadzone to joystick input."""
        deadzone = deadzone or self.deadzone
        return 0.0 if abs(value) < deadzone else value
    
    def process_gamepad_input(self, raw_control):
        """Process gamepad input exactly like the original dora-franka-mujoco."""
        axes = raw_control["axes"]
        buttons = raw_control["buttons"]

        # Reset position with START button
        if len(buttons) > 9 and buttons[9]:
            # Reset to home position and update target
            self.data.qpos[:7] = self.home_pos
            mujoco.mj_forward(self.model, self.data)
            self.target_pos = self.data.body(self.hand_id).xpos.copy()
            print("Reset to home position")
        
        # Gripper control with X and Y buttons (exactly like original)
        if len(buttons) > 0 and buttons[0]:  # X button - Close gripper
            self.data.ctrl[self.gripper_actuator] = self.gripper_range[0]  
            self.gripper_state = 0.0
            print("Gripper: CLOSED")
        elif len(buttons) > 3 and buttons[3]:  # Y button - Open gripper
            self.data.ctrl[self.gripper_actuator] = self.gripper_range[1]  
            self.gripper_state = 1.0
            print("Gripper: OPEN")
        
        # Speed scaling with bumpers (LB/RB)
        if len(buttons) > 4 and buttons[4]:  # LB
            self.speed_scale = max(0.1, self.speed_scale - 0.1)
        if len(buttons) > 5 and buttons[5]:  # RB
            self.speed_scale = min(1.0, self.speed_scale + 0.1)
        
        # Get cartesian control inputs with deadzone 
        dx = self.apply_deadzone(axes[0], self.deadzone) * self.speed_scale * 0.1
        dy = -self.apply_deadzone(axes[1], self.deadzone) * self.speed_scale * 0.1
        dz = -self.apply_deadzone(axes[3], self.deadzone) * self.speed_scale * 0.1
        
        # Update target position incrementally
        if abs(dx) > 0 or abs(dy) > 0 or abs(dz) > 0:
            self.target_pos += np.array([dx, dy, dz])
            self.last_input_source = "gamepad"
            
            # Debug output 
            # print(f"Gamepad control: dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f}")
            # print(f"New target: {self.target_pos}")
            # print(f"Speed: {self.speed_scale:.1f}")
            # print(f"Gripper: {'Open' if self.gripper_state > 0.5 else 'Closed'}")


    def process_target_pose(self, pose_array):
        """Process target pose command."""
        if len(pose_array) >= 3:
            self.target_pos = np.array(pose_array[:3])
        
        if len(pose_array) >= 6:
            self.target_rpy = list(pose_array[3:6])
        
        self.last_input_source = "target_pose"
    
    def get_target_rotation_matrix(self):
        """Convert RPY to rotation matrix."""
        roll_rad, pitch_rad, yaw_rad = np.radians(self.target_rpy)
        desired_rot = transform.Rotation.from_euler('ZYX', [yaw_rad, pitch_rad, roll_rad])
        return desired_rot.as_matrix()
    
    def apply_cartesian_control(self, current_joints):
        """Apply Cartesian control using the exact same method as dora-franka-mujoco."""
        if current_joints is None or len(current_joints) < 7:
            return np.concatenate([self.home_pos, [0]])  # Include gripper
        
        # Update our internal model state with current joint positions
        self.data.qpos[:7] = current_joints[:7]
        mujoco.mj_forward(self.model, self.data)
        
        # Get current end-effector pose
        current_ee_pos = self.data.body(self.hand_id).xpos.copy()
        current_ee_rot = self.data.body(self.hand_id).xmat.reshape(3, 3)
        
        # Calculate position error
        pos_error = self.target_pos - current_ee_pos

        # Construct 6D twist (3 position + 3 orientation)
        twist = np.zeros(6)
        twist[:3] = self.Kpos * pos_error / self.integration_dt
        
        # Orientation control
        current_rot = transform.Rotation.from_matrix(current_ee_rot)
        desired_rot = transform.Rotation.from_matrix(self.get_target_rotation_matrix())
        rot_error = (desired_rot * current_rot.inv()).as_rotvec()
        twist[3:] = self.Kori * rot_error / self.integration_dt
        
        # Get Jacobian for the hand body (exactly like dora-franka-mujoco)
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
    
        # Return 8-dimensional control: 7 arm joints + gripper
        full_commands = np.zeros(8)
        full_commands[:7] = new_joints
        full_commands[7] = self.data.ctrl[self.gripper_actuator]  # Current gripper state
    
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
                controller.current_joint_pos = event["value"].to_numpy()
                full_commands = controller.apply_cartesian_control(controller.current_joint_pos)
                
                node.send_output(
                    "joint_commands",
                    pa.array(full_commands, type=pa.float64()),
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
                
            elif event["id"] == "raw_control":
                raw_control = json.loads(event["value"].to_pylist()[0])
                controller.process_gamepad_input(raw_control)
                
            elif event["id"] == "target_pose":
                pose_array = event["value"].to_numpy()
                controller.process_target_pose(pose_array)

if __name__ == "__main__":
    main()