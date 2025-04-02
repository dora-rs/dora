"""TODO: Add docstring."""

import json
import logging
import os
import sys
from typing import Any

import numpy as np
import pyarrow as pa
import torch
from dora import Node

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("dora_gr00t")

# Ensure the Isaac-GR00T submodule is in the Python path
isaac_gr00t_path = os.path.join(os.path.dirname(__file__), "isaac_gr00t")
if os.path.exists(isaac_gr00t_path):
    sys.path.append(isaac_gr00t_path)
    logger.info(f"Added Isaac-GR00T submodule to Python path: {isaac_gr00t_path}")
else:
    logger.warning(f"Isaac-GR00T submodule not found at {isaac_gr00t_path}")

try:
    # Try to import from the submodule or installed package
    from gr00t.experiment.data_config import DATA_CONFIG_MAP
    from gr00t.model.policy import Gr00tPolicy
    logger.info("Successfully imported Isaac-GR00T")
except ImportError as e:
    logger.warning(f"Could not import Isaac-GR00T: {e}")
    logger.warning("Make sure Isaac-GR00T is installed or available as a git submodule.")
    logger.warning("Using mock implementation for testing purposes.")
    
    # Define mock implementations for testing
    DATA_CONFIG_MAP = {
        "gr1_arms_waist": None,
    }
    
    class Gr00tPolicy:
        """TODO: Add docstring."""

        def __init__(self, model_path=None, modality_config=None, modality_transform=None, 
                     embodiment_tag=None, denoising_steps=4):
            """TODO: Add docstring."""
            self.model_path = model_path
            self.modality_config = modality_config
            self.modality_transform = modality_transform
            self.embodiment_tag = embodiment_tag
            self.denoising_steps = denoising_steps
            logger.info(f"Mock Gr00tPolicy initialized with model_path={model_path}")
            
        def get_action(self, inputs):
            """TODO: Add docstring."""
            logger.info(f"Mock get_action called with inputs: {list(inputs.keys())}")
            return {
                "action_pred": torch.zeros((1, 16, 22))  # Mock action prediction tensor
            }
            
        def apply_transforms(self, inputs):
            """TODO: Add docstring."""
            return inputs
            
        def unapply_transforms(self, outputs):
            """TODO: Add docstring."""
            return {
                "action.left_arm": np.zeros((16, 7)),
                "action.right_arm": np.zeros((16, 7)),
                "action.left_hand": np.zeros((16, 6)),
                "action.right_hand": np.zeros((16, 6)),
                "action.waist": np.zeros((16, 3)),
            }
        
        def _check_state_is_batched(self, obs):
            """TODO: Add docstring."""
            for k, v in obs.items():
                if "state" in k and isinstance(v, np.ndarray) and len(v.shape) < 3:  # (B, Time, Dim)
                    return False
            return True


# Helper functions
def unsqueeze_dict_values(data: dict[str, Any]) -> dict[str, Any]:
    """TODO: Add docstring."""
    unsqueezed_data = {}
    for k, v in data.items():
        if isinstance(v, np.ndarray):
            unsqueezed_data[k] = np.expand_dims(v, axis=0)
        elif isinstance(v, torch.Tensor):
            unsqueezed_data[k] = v.unsqueeze(0)
        else:
            unsqueezed_data[k] = v
    return unsqueezed_data


def squeeze_dict_values(data: dict[str, Any]) -> dict[str, Any]:
    """TODO: Add docstring."""
    squeezed_data = {}
    for k, v in data.items():
        if isinstance(v, np.ndarray):
            squeezed_data[k] = np.squeeze(v, axis=0)
        elif isinstance(v, torch.Tensor):
            squeezed_data[k] = v.squeeze(0)
        else:
            squeezed_data[k] = v
    return squeezed_data


class DoraGr00tNode:
    """TODO: Add docstring."""
    
    def __init__(
        self,
        node: Node,
        model_path: str = "nvidia/GR00T-N1-2B",
        embodiment_tag: str = "gr1",
        data_config: str = "gr1_arms_waist",
        denoising_steps: int = 4,
    ):
        """TODO: Add docstring."""
        self.node = node
        self.model_path = model_path
        self.embodiment_tag = embodiment_tag
        self.data_config = data_config
        self.denoising_steps = denoising_steps
        self.model = None
        
        try:
            logger.info(f"Initializing GR00T model: {self.model_path}")
            
            # Set up data config and transform
            data_config_obj = DATA_CONFIG_MAP[self.data_config]
            modality_config = data_config_obj.modality_config() if data_config_obj else {}
            modality_transform = data_config_obj.transform() if data_config_obj else None
            
            # Create policy
            self.model = Gr00tPolicy(
                model_path=self.model_path,
                modality_config=modality_config,
                modality_transform=modality_transform,
                embodiment_tag=self.embodiment_tag,
                denoising_steps=self.denoising_steps,
            )
            logger.info("GR00T model initialized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize GR00T model: {e}")
            logger.warning("Using mock implementation instead")
            self.model = Gr00tPolicy(
                model_path=self.model_path,
                embodiment_tag=self.embodiment_tag,
                denoising_steps=self.denoising_steps
            )
    
    def process_inputs(self, inputs: dict[str, Any]) -> dict[str, Any]:
        """TODO: Add docstring."""
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
    
    def get_action(self, inputs: dict[str, Any]) -> dict[str, np.ndarray]:
        """TODO: Add docstring."""
        if self.model is None:
            logger.warning("GR00T model not initialized. Using fallback empty actions.")
            return {
                "action.left_arm": np.zeros((16, 7)),
                "action.right_arm": np.zeros((16, 7)),
                "action.left_hand": np.zeros((16, 6)),
                "action.right_hand": np.zeros((16, 6)),
                "action.waist": np.zeros((16, 3)),
            }
        
        try:
            # Convert Dora inputs to the format expected by GR00T
            observations = self.process_inputs(inputs)
            
            # Call GR00T policy's get_action method, which handles:
            # 1. Batch dimension check and handling
            # 2. Applying transforms
            # 3. Model inference
            # 4. Unapplying transforms to get final actions
            return self.model.get_action(observations)
        
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
                input_ids = [
                    "video.ego_view",
                    "state.left_arm", 
                    "state.right_arm", 
                    "state.left_hand", 
                    "state.right_hand", 
                    "state.waist",
                    "annotation.human.action.task_description"
                ]
                
                try:
                    for input_id in input_ids:
                        input_event = self.node.get_input(input_id)
                        if input_event:
                            inputs[input_id] = input_event["value"]
                except Exception as e:
                    logger.error(f"Error getting inputs: {e}")
                
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
    model_path = config.get("model_path", "nvidia/GR00T-N1-2B")
    embodiment_tag = config.get("embodiment_tag", "gr1")
    data_config = config.get("data_config", "gr1_arms_waist")
    denoising_steps = int(config.get("denoising_steps", 4))
    
    logger.info(f"Starting dora-gr00t node with config: {json.dumps(config, indent=2)}")
    
    # Initialize the GR00T node
    gr00t_node = DoraGr00tNode(
        node=node,
        model_path=model_path,
        embodiment_tag=embodiment_tag,
        data_config=data_config,
        denoising_steps=denoising_steps,
    )
    
    # Register the event handler
    node.on_event(gr00t_node.process_event)
    
    try:
        # Start the node
        logger.info("Starting dora-gr00t node")
        node.start()
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received, shutting down...")
    except Exception as e:
        logger.error(f"Error starting node: {e}")
    finally:
        logger.info("dora-gr00t node shut down")


if __name__ == "__main__":
    main()