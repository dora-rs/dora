"""MuJoCo simulation node for Dora with robot descriptions support."""

import numpy as np
import pyarrow as pa
import mujoco
import mujoco.viewer
from dora import Node
import json
import os
from pathlib import Path
import time
from typing import Dict, Any
from robot_descriptions.loaders.mujoco import load_robot_description


class MuJoCoSimulator:
    """MuJoCo simulator for Dora."""
    
    def __init__(self, model_path_or_name: str = None):
        """Initialize the MuJoCo simulator."""
        # Check environment variable first, then use parameter, then default
        self.model_path_or_name = (
            os.getenv("MODEL_NAME") or 
            model_path_or_name or 
            "go2_mj_description"
        )
        
        self.model = None
        self.data = None
        self.viewer = None
        self.state_data = {}
        self.load_model()
        
        print(f"MuJoCo Simulator initialized with model: {self.model_path_or_name}")

    def load_model(self) -> bool:
        """Load MuJoCo model from path or robot description name."""
        model_path = Path(self.model_path_or_name)
        if model_path.exists() and model_path.suffix == '.xml':
            print(f"Loading model from direct path: {model_path}")
            self.model = mujoco.MjModel.from_xml_path(str(model_path))
        else:
            self.model = load_robot_description(self.model_path_or_name, variant="scene")

        # Initialize simulation data
        self.data = mujoco.MjData(self.model)
        
        # Set control to neutral position
        if self.model.nkey > 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        else:
            mujoco.mj_resetData(self.model, self.data)
        
        # Print model info for debugging
        print("Model loaded successfully:")
        print(f"  DOF (nq): {self.model.nq}")
        print(f"  Velocities (nv): {self.model.nv}")
        print(f"  Actuators (nu): {self.model.nu}")
        print(f"  Control inputs: {len(self.data.ctrl)}")
        
        # Print actuator info
        if self.model.nu > 0:
            print("Actuators:")
            for i in range(self.model.nu):
                actuator_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
                joint_id = self.model.actuator_trnid[i, 0]  # First transmission joint
                joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id) if joint_id >= 0 else "N/A"
                ctrl_range = self.model.actuator_ctrlrange[i]
                print(f"  [{i}] {actuator_name or f'actuator_{i}'} -> joint: {joint_name} | range: [{ctrl_range[0]:.2f}, {ctrl_range[1]:.2f}]")
            
        # Initialize state data
        self._update_state_data()
        return True

    def apply_control(self, control_input: np.ndarray):
        """Apply control input to the simulation.

        Args:
            control_input: Control values for actuators

        """
        if control_input is None or len(control_input) == 0:
            return
            
        # Ensure we don't exceed the number of actuators
        n_controls = min(len(control_input), self.model.nu)
        
        # Apply control directly to actuators 
        for i in range(n_controls):
            # Apply joint limits if available
            ctrl_range = self.model.actuator_ctrlrange[i]
            if ctrl_range[0] < ctrl_range[1]:  # Valid range
                control_value = np.clip(control_input[i], ctrl_range[0], ctrl_range[1])
            else:
                control_value = control_input[i]
            
            self.data.ctrl[i] = control_value

    def _get_available_models(self) -> dict:
        """Get available models from the mapping file."""
        config_path = Path(__file__).parent / "robot_models.json"
        with open(config_path) as f:
            return json.load(f)

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
    
    # Initialize simulator
    simulator = MuJoCoSimulator()
    
    # Load model
    if not simulator.load_model():
        print("Failed to load MuJoCo model")
        return
        
    print("MuJoCo simulation node started")
    
    # Launch viewer
    with mujoco.viewer.launch_passive(simulator.model, simulator.data) as viewer:        
        print("MuJoCo viewer launched - simulation running")
        
        # Main Dora event loop
        for event in node:
            if event["type"] == "INPUT":
                # Handle control input
                if event["id"] == "control_input":
                    control_array = event["value"].to_numpy()
                    simulator.apply_control(control_array)
                    # print(f"Applied control: {control_array[:7]}")  # Show first 7 values
            
            # Step simulation once per loop iteration
            simulator.step_simulation()
            viewer.sync()
            
            # Send outputs after stepping
            if event["type"] == "INPUT":
                state = simulator.get_state()
                current_time = state.get("time", time.time())
                
                # Send joint positions
                node.send_output(
                    "joint_positions", 
                    pa.array(state["qpos"]), 
                    {"timestamp": current_time, "encoding": "jointstate"}
                )
                
                # Send joint velocities
                node.send_output(
                    "joint_velocities", 
                    pa.array(state["qvel"]), 
                    {"timestamp": current_time}
                )
                
                # Send actuator controls
                node.send_output(
                    "actuator_controls",
                    pa.array(state["ctrl"]),
                    {"timestamp": current_time}
                )
                
                # Send sensor data if available
                if len(state["sensordata"]) > 0:
                    node.send_output(
                        "sensor_data", 
                        pa.array(state["sensordata"]), 
                        {"timestamp": current_time}
                    )


if __name__ == "__main__":
    main()