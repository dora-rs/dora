import numpy as np
import time
import pyarrow as pa
from dora import Node
from scipy.spatial.transform import Rotation

class FrankaController:
    def __init__(self):
        """
        Initialize the Franka controller
        """
        # Control parameters
        self.integration_dt = 0.1
        self.damping = 1e-4
        self.Kpos = 0.95  # Position gain
        self.Kori = 0.95  # Orientation gain
        self.max_angvel = 3.14 # Max joint velocity (rad/s)
        
        # State variables
        self.dof = None 
        self.current_joint_pos = None  # Full robot state
        self.home_pos = None  # Home position for arm joints only
        
        self.target_pos = np.array([0.4, 0.0, 0.3])  # Conservative default target
        self.target_rpy = [180.0, 0.0, 90.0]  # Downward orientation
        
        self.current_ee_pose = None  # End-effector pose
        self.current_jacobian = None  # Jacobian matrix
        
        print("FrankaController initialized")
    
    def _initialize_robot(self, positions, jacobian_dof=None):
        """Initialize controller state with appropriate dimensions"""
        self.full_joint_count = len(positions)
        # Set DOF from Jacobian if available
        self.dof = jacobian_dof if jacobian_dof is not None else self.full_joint_count

        self.current_joint_pos = positions.copy()
        self.home_pos = np.zeros(self.dof)
    
    def get_target_rotation_matrix(self):
        """Convert RPY to rotation matrix."""
        roll_rad, pitch_rad, yaw_rad = np.radians(self.target_rpy)
        desired_rot = Rotation.from_euler('ZYX', [yaw_rad, pitch_rad, roll_rad])
        return desired_rot.as_matrix()
    
    def set_target_pose(self, pose_array):
        """Set target pose from input array."""
        self.target_pos = np.array(pose_array[:3])
        if len(pose_array) == 6:
            self.target_rpy = list(pose_array[3:6])
        else:
            self.target_rpy = [180.0, 0.0, 90.0] # Default orientation if not provided
    
    def update_jacobian(self, jacobian_flat, shape):
        """Update current jacobian and initialize DOF if needed."""
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
        """Apply differential IK control with nullspace projection."""
        if self.current_ee_pose is None or self.current_jacobian is None:
            return self.current_joint_pos
            
        current_ee_pos = self.current_ee_pose['position']
        current_ee_rpy = self.current_ee_pose['rpy']

        # Calculate position and orientation error
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
        # # Calculate nullspace projection # uncomment to enable nullspace control
        
        # current_arm = self.current_joint_pos[:self.dof]
        # jac_pinv = np.linalg.pinv(jac)
        # N = np.eye(self.dof) - jac_pinv @ jac
        
        # # Apply gains to pull towards home position
        # k_null = np.ones(self.dof) * 5.0
        # dq_null = k_null * (self.home_pos - current_arm)  # Nullspace velocity
        # dq += N @ dq_null  # Nullspace movement
        
        # Limit joint velocities
        dq_abs_max = np.abs(dq).max()
        if dq_abs_max > self.max_angvel:
            dq *= self.max_angvel / dq_abs_max
            
        # Create full joint command (apply IK result to arm joints, keep other joints unchanged)
        new_joints = self.current_joint_pos.copy()
        new_joints[:self.dof] += dq * self.integration_dt
        
        return new_joints

def main():
    node = Node()
    controller = FrankaController()
    
    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "joint_positions":
                joint_pos = event["value"].to_numpy()

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
                joint_commands = controller.apply_differential_ik_control()
                
                # Send control commands to the robot
                node.send_output(
                    "joint_commands", 
                    pa.array(joint_commands, type=pa.float32()),
                    metadata={"timestamp": time.time()}
                )

            # Handle target pose updates
            if event["id"] == "target_pose":
                target_pose = event["value"].to_numpy()
                controller.set_target_pose(target_pose)
            
            # Handle FK results
            if event["id"] == "fk_result":
                if event["metadata"]["encoding"] == "xyzrpy":
                    ee_pose = event["value"].to_numpy()
                    controller.current_ee_pose = {'position': ee_pose[:3],'rpy': ee_pose[3:6]}
            
            # Handle Jacobian results
            if event["id"] == "jacobian_result":
                if event["metadata"]["encoding"] == "jacobian_result":
                    jacobian_flat = event["value"].to_numpy()
                    jacobian_shape = event["metadata"]["jacobian_shape"]
                    controller.update_jacobian(jacobian_flat, jacobian_shape)

if __name__ == "__main__":
    main()