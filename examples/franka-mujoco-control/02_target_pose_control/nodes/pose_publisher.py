"""Target pose publisher node.

This node demonstrates how to send target poses programmatically.
It cycles through predefined poses to show the robot moving.
"""

import time
import numpy as np
import pyarrow as pa
from dora import Node

class PosePublisher:
    """Publishes target poses in sequence."""
    
    def __init__(self):
        # Define a sequence of target poses [x, y, z, roll, pitch, yaw]
        self.target_poses = [
            [0.5, 0.5, 0.3, 180.0, 0.0, 90.0],   
            [0.6, 0.2, 0.5, 180.0, 0.0, 45.0],  
            [0.7, 0.1, 0.4, 90.0, 90.0, 90.0], 
            [0.5, -0.3, 0.6, 180.0, 0.0, 135.0],   
            [-0.3, -0.7, 0.2, 180.0, 0.0, 90.0],   
        ]
        
        self.current_pose_index = 0
        print("Pose Publisher initialized")
        print(f"Will cycle through {len(self.target_poses)} target poses")
    
    def get_next_pose(self):
        """Get the next target pose in sequence."""
        pose = self.target_poses[self.current_pose_index]
        self.current_pose_index = (self.current_pose_index + 1) % len(self.target_poses)
        return pose

def main():
    node = Node("pose_publisher")
    publisher = PosePublisher()
    
    print("Pose Publisher Node Started")
    print("Publishing target poses every 10 seconds...")
    # time.sleep(10)  # Allow time for node to initialize
    for event in node:
        if event["type"] == "INPUT" and event["id"] == "tick":
            # Get next target pose
            target_pose = publisher.get_next_pose()
            
            print(f"Publishing target pose: {target_pose}")
            
            # Send target pose
            node.send_output(
                "target_pose",
                pa.array(target_pose, type=pa.float64()),
                metadata={"timestamp": time.time()}
            )

if __name__ == "__main__":
    main()