import json
import time
import numpy as np
import pyarrow as pa
from dora import Node

class GamepadController:
    def __init__(self):
        """
        Initialize the simple gamepad controller
        """
        # Robot state variables
        self.dof = None 
        self.current_joint_pos = None
        
        # Target pose (independent of DOF)
        self.target_pos = np.array([0.4, 0.0, 0.3])  # Conservative default target
        self.target_rpy = [180.0, 0.0, 90.0]  # Downward orientation
        self.ik_request_sent = True 

        
        print("Simple Gamepad Controller initialized")
    
    def _initialize_robot(self, positions):
        """Initialize controller state with appropriate dimensions"""
        self.full_joint_count = len(positions)
        self.current_joint_pos = positions.copy()
        if self.dof is None:
            self.dof = self.full_joint_count
    
    def process_cmd_vel(self, cmd_vel):
        """Process gamepad velocity commands to update target position"""
        delta = cmd_vel[:3] * 0.03
        dx, dy, dz = delta
        
        # Update target position incrementally
        if abs(dx) > 0 or abs(dy) > 0 or abs(dz) > 0:
            self.target_pos += np.array([dx, -dy, dz])
            self.ik_request_sent = False 

    
    def process_gamepad_input(self, raw_control):
        """Process gamepad button inputs"""
        buttons = raw_control["buttons"]

        # Reset position with START button
        if len(buttons) > 9 and buttons[9]:
            self.target_pos = np.array([0.4, 0.0, 0.3])
            print("Reset target to home position")
            self.ik_request_sent = False 
    
    def get_target_pose_array(self):
        """Get target pose as 6D array [x, y, z, roll, pitch, yaw]"""
        return np.concatenate([self.target_pos, self.target_rpy])

def main():
    node = Node()
    controller = GamepadController()
    
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
                
            elif event["id"] == "raw_control":
                raw_control = json.loads(event["value"].to_pylist()[0])
                controller.process_gamepad_input(raw_control)
                
            elif event["id"] == "cmd_vel":
                cmd_vel = event["value"].to_numpy()
                controller.process_cmd_vel(cmd_vel)
            
            # Handle IK results and send directly as joint commands
            elif event["id"] == "ik_request":
                if event["metadata"]["encoding"] == "jointstate":
                    ik_solution = event["value"].to_numpy()
                    # Send IK solution directly as joint commands
                    node.send_output(
                        "joint_commands", 
                        pa.array(ik_solution, type=pa.float32()),
                        metadata={"timestamp": time.time()}
                    )

if __name__ == "__main__":
    main()