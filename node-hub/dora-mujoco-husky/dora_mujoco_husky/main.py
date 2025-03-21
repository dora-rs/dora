from dora import Node
import mujoco
import mujoco.viewer
import pyarrow as pa
import os

def clamp(value, min_val, max_val):
    """Clamp value between min_val and max_val"""
    return max(min(value, max_val), min_val)

def main():
    try:
        # Initialize node
        node = Node("mujoco_husky")
        print("Initializing Mujoco Husky simulation...")
        
        # Load Husky model
        model_path = os.path.join(os.path.dirname(__file__), "husky/husky.xml")
        model = mujoco.MjModel.from_xml_path(model_path)
        data = mujoco.MjData(model)
        
        # Define velocity limits
        MAX_LINEAR_VELOCITY = 2.0   
        MAX_ANGULAR_VELOCITY = 3.0  
        
        # Initialize viewer
        with mujoco.viewer.launch_passive(model, data) as viewer:
            print("Simulation initialized successfully")

            for event in node:
                if event["type"] == "INPUT" and event["id"] == "cmd_vel":
                    try:
                        cmd_vel = event["value"].to_pylist()

                        linear_x = clamp(float(cmd_vel[0]), -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY)
                        angular_z = clamp(float(cmd_vel[5]), -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY)
                        
                        print("husky cmd_vel: ", linear_x, angular_z)  # Debug

                        wheel_radius = 0.17775  # Actual Husky wheel radius
                        wheel_separation = 0.59  # Actual Husky track width
                        
                        left_velocity = ((linear_x - angular_z * wheel_separation / 2.0) / wheel_radius) / 3.0  # added these to limit the huskys speed
                        right_velocity = ((linear_x + angular_z * wheel_separation / 2.0) / wheel_radius) / 3.0
                        
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
                        
                    except Exception as e:
                        print(f"Error processing cmd_vel: {e}")
                        raise e

    except KeyboardInterrupt:
        print("\nExiting simulation...")
    except Exception as e:
        print(f"Simulation error: {e}")
        raise e

if __name__ == "__main__":
    main()