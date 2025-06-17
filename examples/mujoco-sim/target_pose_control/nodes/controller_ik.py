import numpy as np
import time
import pyarrow as pa
from dora import Node

class Controller:
    def __init__(self):
        """
        Initialize the simple controller
        """
        # State variables
        self.dof = None 
        self.current_joint_pos = None
        
        self.target_pos = np.array([0.4, 0.0, 0.3])  # Conservative default target
        self.target_rpy = [180.0, 0.0, 90.0]  # Downward orientation
        self.ik_request_sent = True 
        
        print("Simple Controller initialized")
    
    def _initialize_robot(self, positions):
        """Initialize controller state with appropriate dimensions"""
        self.full_joint_count = len(positions)
        self.current_joint_pos = positions.copy()
        if self.dof is None:
            self.dof = self.full_joint_count
    
    def set_target_pose(self, pose_array):
        """Set target pose from input array."""
        self.target_pos = np.array(pose_array[:3])
        if len(pose_array) == 6:
            self.target_rpy = list(pose_array[3:6])
        else:
            self.target_rpy = [180.0, 0.0, 90.0] # Default orientation if not provided
        self.ik_request_sent = False
    
    def get_target_pose_array(self):
        """Get target pose as 6D array [x, y, z, roll, pitch, yaw]"""
        return np.concatenate([self.target_pos, self.target_rpy])

def main():
    node = Node()
    controller = Controller()
    
    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "joint_positions":
                joint_pos = event["value"].to_numpy()

                if controller.current_joint_pos is None:
                    controller._initialize_robot(joint_pos)
                else:
                    controller.current_joint_pos = joint_pos
                
                # Request IK solution directly
                target_pose = controller.get_target_pose_array()
                if not controller.ik_request_sent:
                    node.send_output(
                        "ik_request", 
                        pa.array(target_pose, type=pa.float32()),
                        metadata={"encoding": "xyzrpy", "timestamp": time.time()}
                    )
                    controller.ik_request_sent = True
                
                node.send_output(
                    "joint_state", 
                    pa.array(joint_pos, type=pa.float32()),
                    metadata={"encoding": "jointstate", "timestamp": time.time()}
                )

            # Handle target pose updates
            if event["id"] == "target_pose":
                target_pose = event["value"].to_numpy()
                controller.set_target_pose(target_pose)
            
            # Handle IK results and send directly as joint commands
            if event["id"] == "ik_request":
                if event["metadata"]["encoding"] == "jointstate":
                    ik_solution = event["value"].to_numpy()
                    # print("ik_solution", ik_solution)
                    # Send IK solution directly as joint commands
                    node.send_output(
                        "joint_commands", 
                        pa.array(ik_solution, type=pa.float32()),
                        metadata={"timestamp": time.time()}
                    )

if __name__ == "__main__":
    main()