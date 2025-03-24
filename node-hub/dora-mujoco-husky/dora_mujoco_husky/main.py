"""MuJoCo Husky simulation node for Dora.

This module provides a Dora node that simulates a Husky robot using the MuJoCo physics engine.
It handles velocity commands and publishes position and velocity feedback.
"""

import os

import mujoco
import mujoco.viewer
import pyarrow as pa
from dora import Node

from .mesh_downloader import ensure_meshes


def clamp(value, min_val, max_val):
    """Clamp value between min_val and max_val."""
    return max(min(value, max_val), min_val)

def main():
    try:
        node = Node("mujoco_husky")
        print("Initializing Mujoco Husky simulation...")
        
        ensure_meshes()
        
        model_path = os.path.join(os.path.dirname(__file__), "husky/husky.xml")
        model = mujoco.MjModel.from_xml_path(model_path)
        data = mujoco.MjData(model)
        
        # Define velocity limits
        max_linear_velocity = 2.0   
        max_angular_velocity = 3.0  
        
        with mujoco.viewer.launch_passive(model, data) as viewer:
            print("Simulation initialized successfully")

            for event in node:
                if event["type"] == "INPUT" and event["id"] == "cmd_vel":

                    cmd_vel = event["value"].to_pylist()

                    linear_x = clamp(float(cmd_vel[0]), -max_linear_velocity, max_linear_velocity)
                    angular_z = clamp(float(cmd_vel[5]), -max_angular_velocity, max_angular_velocity)

                    wheel_radius = 0.17775  # Actual Husky wheel radius
                    wheel_separation = 0.59  # Actual Husky track width
                    
                    left_velocity = ((linear_x - angular_z * wheel_separation / 2.0) / wheel_radius)
                    right_velocity = ((linear_x + angular_z * wheel_separation / 2.0) / wheel_radius)
                    
                    # Apply wheel velocities
                    data.ctrl[0] = left_velocity   
                    data.ctrl[1] = right_velocity    
                    data.ctrl[2] = left_velocity   
                    data.ctrl[3] = right_velocity  
                    
                    
                    mujoco.mj_step(model, data)
                    viewer.sync()
                    
                    # Send position and velocity data or any other form of feedback from mujoco
                    position = data.qpos[:3].tolist()
                    velocity = data.qvel[:3].tolist()
                    
                    node.send_output("position", 
                        data=pa.array(position, type=pa.float64()),
                        metadata={"type": "position"}
                    )
                    
                    node.send_output("velocity",
                        data=pa.array(velocity, type=pa.float64()),
                        metadata={"type": "velocity"}
                    )

    except KeyboardInterrupt:
        print("\nExiting simulation...")
    except Exception as e:
        print(f"Simulation error: {e}")
        raise e

if __name__ == "__main__":
    main()