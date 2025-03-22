"""TODO: Add docstring."""

import os
import sys
import pyarrow as pa
import numpy as np
import time
import json
import threading
import logging
from typing import Dict, Any
from dora import Node

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("dora_gr00t")

# Ensure the Isaac-GR00T submodule is in the Python path
isaac_gr00t_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "isaac_gr00t")
if os.path.exists(isaac_gr00t_path):
    sys.path.append(isaac_gr00t_path)
    logger.info(f"Added Isaac-GR00T submodule to Python path: {isaac_gr00t_path}")
else:
    logger.warning(f"Isaac-GR00T submodule not found at {isaac_gr00t_path}")

try:
    # Try to import from the submodule or installed package
    from gr00t.eval.robot import RobotInferenceClient, RobotInferenceServer
    from gr00t.experiment.data_config import DATA_CONFIG_MAP
    from gr00t.model.policy import Gr00tPolicy
    logger.info("Successfully imported Isaac-GR00T")
except ImportError as e:
    logger.warning(f"Could not import Isaac-GR00T: {e}")
    logger.warning("Make sure Isaac-GR00T is installed or available as a git submodule.")
    logger.warning("Using mock implementation for testing purposes.")
    
    # Define mock implementations for testing
    class RobotInferenceClient:
        """TODO: Add docstring."""

        def __init__(self, host="localhost", port=5555):
            logger.info(f"Mock RobotInferenceClient initialized with host={host}, port={port}")
            self.host = host
            self.port = port
        
        def get_modality_config(self):
            """TODO: Add docstring."""
            return {
                "video.ego_view": {"shape": (1, 256, 256, 3), "dtype": "uint8"},
                "state.left_arm": {"shape": (1, 7), "dtype": "float32"},
                "state.right_arm": {"shape": (1, 7), "dtype": "float32"},
                "state.left_hand": {"shape": (1, 6), "dtype": "float32"},
                "state.right_hand": {"shape": (1, 6), "dtype": "float32"},
                "state.waist": {"shape": (1, 3), "dtype": "float32"},
                "annotation.human.action.task_description": {"dtype": "string"},
            }
        
        def get_action(self, inputs):
            """TODO: Add docstring."""
            logger.info(f"Mock get_action called with inputs: {list(inputs.keys())}")
            return {
                "action.left_arm": np.zeros((16, 7)),
                "action.right_arm": np.zeros((16, 7)),
                "action.left_hand": np.zeros((16, 6)),
                "action.right_hand": np.zeros((16, 6)),
                "action.waist": np.zeros((16, 3)),
            }
    
    class RobotInferenceServer:
        """TODO: Add docstring."""
        
        def __init__(self, policy, port=5555):
            self.policy = policy
            self.port = port
            logger.info(f"Mock RobotInferenceServer initialized with port={port}")
            
        def run(self):
            """TODO: Add docstring."""
            logger.info("Mock RobotInferenceServer running")
    
    DATA_CONFIG_MAP = {
        "gr1_arms_waist": None,
    }
    
    class Gr00tPolicy:
        """TODO: Add docstring."""

        def __init__(self, model_path=None, modality_config=None, modality_transform=None, 
                     embodiment_tag=None, denoising_steps=4):
            self.model_path = model_path
            self.modality_config = modality_config
            self.modality_transform = modality_transform
            self.embodiment_tag = embodiment_tag
            self.denoising_steps = denoising_steps
            logger.info(f"Mock Gr00tPolicy initialized with model_path={model_path}")


class DoraGr00tNode:
    """TODO: Add docstring."""
    
    def __init__(
        self,
        node: Node,
        host: str = "localhost",
        port: int = 5555,
        connection_retry_attempts: int = 5,
        retry_interval: int = 2,
        model_path: str = "nvidia/GR00T-N1-2B",
        embodiment_tag: str = "gr1",
        data_config: str = "gr1_arms_waist",
        denoising_steps: int = 4,
        server_mode: bool = False,
    ):
        """TODO: Add docstring."""
        self.node = node
        self.host = host
        self.port = port
        self.model_path = model_path
        self.embodiment_tag = embodiment_tag
        self.data_config = data_config
        self.denoising_steps = denoising_steps
        self.server_mode = server_mode
        self.client = None
        self.server = None
        
        # If running in server mode, start a GR00T server
        if self.server_mode:
            self._start_server()
        
        # Try to connect to the GR00T inference server
        for i in range(connection_retry_attempts):
            try:
                self.client = RobotInferenceClient(host=self.host, port=self.port)
                logger.info(f"Successfully connected to GR00T inference server at {host}:{port}")
                break
            except Exception as e:
                logger.warning(f"Connection attempt {i+1}/{connection_retry_attempts} failed: {e}")
                if i < connection_retry_attempts - 1:
                    logger.info(f"Retrying in {retry_interval} seconds...")
                    time.sleep(retry_interval)
                else:
                    logger.warning("Failed to connect to GR00T inference server. Using mock implementation instead.")
                    self.client = RobotInferenceClient(host=self.host, port=self.port)
        
        # Get available modality configs
        if self.client:
            try:
                self.modality_configs = self.client.get_modality_config()
                logger.info("Available modality configs:")
                logger.info(list(self.modality_configs.keys()))
            except Exception as e:
                logger.error(f"Error getting modality configs: {e}")
                self.modality_configs = {}
    
    def _start_server(self):
        """TODO: Add docstring."""
        try:
            logger.info(f"Starting GR00T server with model: {self.model_path}")
            
            # Set up data config and transform
            data_config = DATA_CONFIG_MAP[self.data_config]
            modality_config = data_config.modality_config() if data_config else {}
            modality_transform = data_config.transform() if data_config else None
            
            # Create policy
            policy = Gr00tPolicy(
                model_path=self.model_path,
                modality_config=modality_config,
                modality_transform=modality_transform,
                embodiment_tag=self.embodiment_tag,
                denoising_steps=self.denoising_steps,
            )
            
            # Start server in a separate thread
            def run_server():
                """TODO: Add docstring."""
                self.server = RobotInferenceServer(policy, port=self.port)
                self.server.run()
            
            self.server_thread = threading.Thread(target=run_server, daemon=True)
            self.server_thread.start()
            logger.info(f"GR00T server started on port {self.port}")
            
            # Give the server a moment to start
            time.sleep(2)
        except Exception as e:
            logger.error(f"Failed to start GR00T server: {e}")
    
    def process_inputs(self, inputs: Dict[str, Any]) -> Dict[str, Any]:
        """
        Process dora inputs into the format expected by GR00T.
        
        Args:
            inputs: Dictionary of dora inputs
            
        Returns:
            Dictionary of formatted inputs for GR00T
        """
        # Map dora inputs to GR00T input format
        gr00t_inputs = {}
        
        # Video input (expects shape [1, 256, 256, 3])
        if "video.ego_view" in inputs:
            video_data = inputs["video.ego_view"].to_numpy()
            # Ensure correct shape for GR00T
            if len(video_data.shape) == 3:  # [H, W, C]
                video_data = np.expand_dims(video_data, axis=0)  # Add batch dimension [1, H, W, C]
            # Ensure correct data type
            if video_data.dtype != np.uint8:
                video_data = video_data.astype(np.uint8)
            gr00t_inputs["video.ego_view"] = video_data
            logger.debug(f"Processed video input with shape {video_data.shape}")
        
        # State inputs
        for input_key in [
            "state.left_arm",
            "state.right_arm",
            "state.left_hand",
            "state.right_hand",
            "state.waist"
        ]:
            if input_key in inputs:
                state_data = inputs[input_key].to_numpy()
                # Ensure correct shape with batch dimension
                if len(state_data.shape) == 1:
                    state_data = np.expand_dims(state_data, axis=0)
                # Ensure correct data type
                if state_data.dtype != np.float32:
                    state_data = state_data.astype(np.float32)
                gr00t_inputs[input_key] = state_data
                logger.debug(f"Processed {input_key} input with shape {state_data.shape}")
        
        # Text input
        if "annotation.human.action.task_description" in inputs:
            text_data = inputs["annotation.human.action.task_description"]
            # Ensure it's a list of strings
            if isinstance(text_data, pa.Array):
                text_data = text_data.to_pylist()
            if not isinstance(text_data, list):
                text_data = [str(text_data)]
            gr00t_inputs["annotation.human.action.task_description"] = text_data
            logger.debug(f"Processed text input: {text_data}")
        
        return gr00t_inputs
    
    def get_action(self, inputs: Dict[str, Any]) -> Dict[str, np.ndarray]:
        """TODO: Add docstring."""
        if self.client is None:
            logger.warning("GR00T client not initialized. Using fallback empty actions.")
            return {
                "action.left_arm": np.zeros((16, 7)),
                "action.right_arm": np.zeros((16, 7)),
                "action.left_hand": np.zeros((16, 6)),
                "action.right_hand": np.zeros((16, 6)),
                "action.waist": np.zeros((16, 3)),
            }
        
        processed_inputs = self.process_inputs(inputs)
        
        # Perform prediction
        try:
            actions = self.client.get_action(processed_inputs)
            logger.info(f"Got actions with keys: {list(actions.keys())}")
            return actions
        except Exception as e:
            logger.error(f"Error getting action from GR00T: {e}")
            # Return empty actions as fallback
            return {
                "action.left_arm": np.zeros((16, 7)),
                "action.right_arm": np.zeros((16, 7)),
                "action.left_hand": np.zeros((16, 6)),
                "action.right_hand": np.zeros((16, 6)),
                "action.waist": np.zeros((16, 3)),
            }

    def process_event(self, event):
        """TODO: Add docstring."""
        if event["type"] == "INPUT":
            if event["id"] in [
                "video.ego_view",
                "state.left_arm", 
                "state.right_arm", 
                "state.left_hand", 
                "state.right_hand", 
                "state.waist",
                "annotation.human.action.task_description"
            ]:
                logger.info(f"Received input: {event['id']}")
                
                # Collect all inputs to process them together
                inputs = {}
                for input_id in [
                    "video.ego_view",
                    "state.left_arm", 
                    "state.right_arm", 
                    "state.left_hand", 
                    "state.right_hand", 
                    "state.waist",
                    "annotation.human.action.task_description"
                ]:
                    try:
                        input_event = self.node.get_input(input_id)
                        if input_event:
                            inputs[input_id] = input_event["value"]
                    except Exception as e:
                        logger.error(f"Error getting input {input_id}: {e}")
                
                # Check if we have at least some of the required inputs
                if inputs:
                    try:
                        # Get actions from GR00T
                        actions = self.get_action(inputs)
                        
                        # Send actions as outputs
                        for action_key, action_value in actions.items():
                            if action_key.startswith("action."):
                                output_id = action_key
                                self.node.send_output(
                                    output_id=output_id,
                                    data=pa.array(action_value.tolist()),
                                    metadata={}
                                )
                                logger.info(f"Sent output: {output_id}, shape: {action_value.shape}")
                    except Exception as e:
                        logger.error(f"Error processing GR00T actions: {e}")


def main():
    """TODO: Add docstring."""
    node = Node()
    
    # Get configuration from the node
    config = node.config if hasattr(node, "config") else {}
    
    # Extract configuration parameters with defaults
    host = config.get("host", "localhost")
    port = int(config.get("port", 5555))
    connection_retry_attempts = int(config.get("connection_retry_attempts", 5))
    retry_interval = int(config.get("retry_interval", 2))
    model_path = config.get("model_path", "nvidia/GR00T-N1-2B")
    embodiment_tag = config.get("embodiment_tag", "gr1")
    data_config = config.get("data_config", "gr1_arms_waist")
    denoising_steps = int(config.get("denoising_steps", 4))
    server_mode = config.get("server_mode", False)
    
    logger.info(f"Starting dora-gr00t node with config: {json.dumps(config, indent=2)}")
    
    # Initialize the GR00T node
    gr00t_node = DoraGr00tNode(
        node=node,
        host=host,
        port=port,
        connection_retry_attempts=connection_retry_attempts,
        retry_interval=retry_interval,
        model_path=model_path,
        embodiment_tag=embodiment_tag,
        data_config=data_config,
        denoising_steps=denoising_steps,
        server_mode=server_mode,
    )
    
    # Register the event handler
    node.on_event(gr00t_node.process_event)
    
    # Start the node
    logger.info("Starting dora-gr00t node")
    node.start()


if __name__ == "__main__":
    main()