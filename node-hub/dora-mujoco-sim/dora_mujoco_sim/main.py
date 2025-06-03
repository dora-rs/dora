"""MuJoCo simulation node for Dora with robot descriptions support."""

import numpy as np
import pyarrow as pa
import mujoco
import mujoco.viewer
from dora import Node
import json
from pathlib import Path
import time
from typing import Dict, Any
from robot_descriptions.loaders.mujoco import load_robot_description


class MuJoCoSimulator:
    """MuJoCo simulator for Dora."""
    
    def __init__(self, model_path_or_name: str = "go2"):
        """Initialize the MuJoCo simulator."""
        self.model_path_or_name = model_path_or_name
        self.model = None
        self.data = None
        self.viewer = None
        self.state_data = {}
        self.model_mapping = self._load_model_mapping()
        
    def _load_model_mapping(self) -> dict:
        """Load robot model mapping from JSON file."""
        config_path = Path(__file__).parent / "robot_models.json"

        with open(config_path) as f:
            mapping_data = json.load(f)
        
        model_mapping = {}
        for models in mapping_data.values():
            model_mapping.update(models)
            
        return model_mapping
            

    def load_model(self) -> bool:
        """Load MuJoCo model from path or robot description name."""
        model_path = Path(self.model_path_or_name)
        if model_path.exists() and model_path.suffix == '.xml':
            # print(f"Loading model from direct path: {model_path}")
            self.model = mujoco.MjModel.from_xml_path(str(model_path))
            
        else:
            # Treat as model name - robot_descriptions 
            description_name = self.model_mapping.get(
                self.model_path_or_name, 
                f"{self.model_path_or_name}_mj_description"
            )
            self.model = load_robot_description(description_name, variant="scene")
            
        # Initialize simulation data
        self.data = mujoco.MjData(self.model)
        
        # Set control to neutral position
        if self.model.nkey > 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        else:
            mujoco.mj_resetData(self.model, self.data)
            
        # Initialize state data
        self._update_state_data()
        return True


    def step_simulation(self):
        """Step the simulation forward."""
        mujoco.mj_step(self.model, self.data)
        self._update_state_data()
    
    def _update_state_data(self):
        """Update state data that can be sent via Dora."""
        self.state_data = {
            "time": self.data.time,                    # Current simulation time
            "qpos": self.data.qpos.copy(),            # Joint positions  
            "qvel": self.data.qvel.copy(),            # Joint velocities
            "qacc": self.data.qacc.copy(),            # Joint accelerations
            "ctrl": self.data.ctrl.copy(),            # Control inputs/actuator commands
            "qfrc_applied": self.data.qfrc_applied.copy(),  # External forces applied to joints
            "sensordata": self.data.sensordata.copy() if self.model.nsensor > 0 else np.array([])  # Sensor readings
        }
    
    def get_state(self) -> Dict[str, Any]:
        """Get current simulation state."""
        return self.state_data.copy()
    


def main():
    """Run the main Dora node function."""
    node = Node()
    
    # Configuration - can be either a model name or direct path
    model_path_or_name = "go2"  # Examples: "go2", "franka_panda", "/path/to/scene.xml"
    
    # Initialize simulator
    simulator = MuJoCoSimulator(model_path_or_name)
    
    # Load model
    if not simulator.load_model():
        print("Failed to load MuJoCo model")
        return
        
    print("MuJoCo simulation node started")
    
    # Launch viewer (optional - comment out for headless)
    with mujoco.viewer.launch_passive(simulator.model, simulator.data) as viewer:
        print("Viewer launched")
        
        # Main Dora event loop
        for event in node:
            # Always step simulation
            simulator.step_simulation()
            viewer.sync()
            
            if event["type"] == "INPUT":
                state = simulator.get_state()

                # Get current time
                current_time = state.get("time", time.time())
                
                # Send joint positions
                if "qpos" in state:
                    print(f"Sending joint positions: {state['qpos']}")
                    node.send_output(
                        "joint_positions", 
                        pa.array(state["qpos"]), 
                        {"timestamp": current_time}
                    )
                
                # Send joint velocities
                if "qvel" in state:
                    node.send_output(
                        "joint_velocities", 
                        pa.array(state["qvel"]), 
                        {"timestamp": current_time}
                    )
                
                # Send sensor data if available
                if "sensordata" in state and len(state["sensordata"]) > 0:
                    node.send_output(
                        "sensor_data", 
                        pa.array(state["sensordata"]), 
                        {"timestamp": current_time}
                    )



if __name__ == "__main__":
    main()