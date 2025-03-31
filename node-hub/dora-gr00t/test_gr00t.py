"""TODO: Add docstring."""

import argparse
import logging
import time

import numpy as np

from dora_gr00t.main import DoraGr00tNode

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("test_gr00t")

def generate_dummy_video(width=224, height=224, frames=3, channels=3):
    """TODO: Add docstring."""
    # Use a gradiant pattern so we can see if it's being processed correctly
    video = np.zeros((frames, height, width, channels), dtype=np.uint8)
    for i in range(frames):
        for j in range(height):
            for k in range(width):
                # Create a gradient pattern
                video[i, j, k, 0] = (j * 255) // height  # Red varies with y
                video[i, j, k, 1] = (k * 255) // width   # Green varies with x
                video[i, j, k, 2] = ((j+k) * 255) // (height+width)  # Blue is a diagonal gradient
    return video

def generate_dummy_state(dims):
    """TODO: Add docstring."""
    rng = np.random.default_rng()
    return rng.normal(0, 0.1, (dims,)).astype(np.float32)

def get_dummy_inputs():
    """TODO: Add docstring."""
    inputs = {}
    
    # Video frame
    video_frame = generate_dummy_video()
    inputs["video.ego_view"] = video_frame
    
    # State vectors
    state_dims = {
        "state.left_arm": 7,
        "state.right_arm": 7,
        "state.left_hand": 6,
        "state.right_hand": 6,
        "state.waist": 3
    }
    
    for state_name, dims in state_dims.items():
        inputs[state_name] = generate_dummy_state(dims)
    
    # Text description
    task_descriptions = [
        "Pick up the cup from the table",
        "Move the object to the right",
        "Open the drawer",
        "Close the cabinet door",
        "Reach for the object"
    ]
    inputs["annotation.human.action.task_description"] = task_descriptions[0]
    
    return inputs

class MockNode:
    """TODO: Add docstring."""
    
    def __init__(self):
        """Initialize the mock inference client."""
        self.config = {
            "model_path": "nvidia/GR00T-N1-2B",
            "embodiment_tag": "gr1",
            "data_config": "gr1_arms_waist",
            "denoising_steps": 4
        }
        self.event_handler = None
    
    def on_event(self, handler):
        """TODO: Add docstring."""
        self.event_handler = handler
    
    def get_input(self, input_id):
        """TODO: Add docstring."""
        # This would normally come from the Dora dataflow
        inputs = get_dummy_inputs()
        if input_id in inputs:
            value = inputs[input_id]
            # Convert to PyArrow arrays in a way that can be safely converted back
            if isinstance(value, np.ndarray):
                class MockPyArrowArray:
                    def __init__(self, data):
                        self.data = data
                    
                    def to_numpy(self, zero_copy_only=False):
                        # Ignore zero_copy_only to avoid the PyArrow error
                        return self.data
                
                return {"value": MockPyArrowArray(value)}
            if isinstance(value, str):
                class MockPyArrowArray:
                    def __init__(self, data):
                        self.data = data
                    
                    def to_pylist(self):
                        return [self.data]
                
                return {"value": MockPyArrowArray(value)}
            return {"value": value}
        return None
    
    def send_output(self, output_id, data, metadata=None):
        """TODO: Add docstring."""
        logger.info(f"Sending output: {output_id}, shape: {data.shape if hasattr(data, 'shape') else 'N/A'}")

def test_gr00t_with_dummy_inputs(duration=10, interval=1.0):
    """TODO: Add docstring."""
    logger.info("Creating mock Dora node")
    node = MockNode()
    
    logger.info("Initializing GR00T node")
    gr00t_node = DoraGr00tNode(
        node=node,
        model_path=node.config["model_path"],
        embodiment_tag=node.config["embodiment_tag"],
        data_config=node.config["data_config"],
        denoising_steps=node.config["denoising_steps"]
    )
    
    # Register the event handler
    node.on_event(gr00t_node.process_event)
    
    logger.info("Simulating input events")
    start_time = time.time()
    
    # Send inputs one by one to mimic real-world scenario
    send_interval = 0.1  # seconds

    input_order = [
        "video.ego_view",
        "state.left_arm",
        "state.right_arm",
        "state.left_hand",
        "state.right_hand",
        "state.waist",
        "annotation.human.action.task_description"
    ]
    
    try:
        while time.time() - start_time < duration:
            # Process each input with a small delay between them
            for input_id in input_order:
                # Trigger the GR00T node to process this input
                input_event = {"type": "INPUT", "id": input_id, "timestamp": time.time()}
                gr00t_node.process_event(input_event)
                
                # Small delay between inputs to simulate real-world scenario
                time.sleep(send_interval)
            
            # Wait before sending the next batch of inputs
            time.sleep(interval)
    except KeyboardInterrupt:
        logger.info("Test interrupted by user")
    
    logger.info("Test completed")

def main():
    """TODO: Add docstring."""
    parser = argparse.ArgumentParser(description="Test GR00T node with dummy inputs")
    parser.add_argument("--duration", type=int, default=10, 
                        help="Duration to run the test (in seconds)")
    parser.add_argument("--interval", type=float, default=1.0, 
                        help="Interval between input batches (in seconds)")
    args = parser.parse_args()
    
    test_gr00t_with_dummy_inputs(
        duration=args.duration,
        interval=args.interval
    )

if __name__ == "__main__":
    main()
