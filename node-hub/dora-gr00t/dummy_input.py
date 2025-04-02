"""TODO: Add docstring."""

import argparse
import logging
import threading
import time

import numpy as np
import pyarrow as pa
from dora import Node

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("dummy_input")

class DummyInputNode:
    """TODO: Add docstring."""
    
    def __init__(self, node, interval=1.0):
        """TODO: Add docstring."""
        self.node = node
        self.interval = interval
        self.frame_count = 0
        self.running = True
        
        # Start the generation loop in a separate thread
        self.thread = threading.Thread(target=self._generation_loop)
        self.thread.daemon = True
    
    def start(self):
        """Start generating inputs."""
        logger.info(f"Starting dummy input generation with interval {self.interval}s")
        self.thread.start()
        
    def stop(self):
        """TODO: Add docstring."""
        logger.info("Stopping dummy input generation")
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=2.0)
    
    def _generation_loop(self):
        """TODO: Add docstring."""
        logger.info("Input generation thread started")
        while self.running:
            try:
                self.generate_and_send_inputs()
                time.sleep(self.interval)
            except Exception as e:  # noqa: PERF203
                logger.error(f"Error in generation loop: {e}")
    
    def generate_dummy_video(self):
        """TODO: Add docstring."""
        # Create a color gradient that changes over time
        frame = np.zeros((256, 256, 3), dtype=np.uint8)
        hue = (self.frame_count % 100) / 100.0
        
        # Create a simple color pattern
        for i in range(256):
            for j in range(256):
                r = int(255 * (i/256))
                g = int(255 * (j/256))
                b = int(255 * hue)
                frame[i, j] = [r, g, b]
        
        return frame
    
    def generate_dummy_state(self, dims):
        """TODO: Add docstring."""
        # Generate some sine wave pattern for more realistic-looking motion
        t = self.frame_count / 10.0
        base_values = np.sin(np.linspace(0, 2*np.pi, dims) + t)
        return base_values.astype(np.float32)
    
    def generate_and_send_inputs(self):
        """TODO: Add docstring."""
        self.frame_count += 1
        logger.info(f"Generating dummy inputs (frame {self.frame_count})")
        
        try:
            # Generate video frame
            video_frame = self.generate_dummy_video()
            self.node.send_output(
                output_id="video.ego_view",
                data=pa.array([video_frame.tolist()]),
                metadata={}
            )
            logger.info(f"Sent dummy video frame: shape={video_frame.shape}")
            
            # Generate state vectors
            state_dims = {
                "state.left_arm": 7,
                "state.right_arm": 7,
                "state.left_hand": 6,
                "state.right_hand": 6,
                "state.waist": 3
            }
            
            for state_name, dims in state_dims.items():
                state_data = self.generate_dummy_state(dims)
                self.node.send_output(
                    output_id=state_name,
                    data=pa.array([state_data.tolist()]),
                    metadata={}
                )
                logger.info(f"Sent dummy {state_name}: shape={state_data.shape}")
            
            # Generate text description
            task_descriptions = [
                "Pick up the cup from the table",
                "Move the object to the right",
                "Open the drawer",
                "Close the cabinet door",
                "Reach for the object"
            ]
            task_idx = self.frame_count % len(task_descriptions)
            task_description = task_descriptions[task_idx]
            
            self.node.send_output(
                output_id="annotation.human.action.task_description",
                data=pa.array([task_description]),
                metadata={}
            )
            logger.info(f"Sent dummy task description: {task_description}")
        except Exception as e:
            logger.error(f"Error sending data: {e}")

    def process_event(self, event):
        """TODO: Add docstring."""
        logger.debug(f"Received event: {event}")
        
        # Add specific event handling if needed
        if event["type"] == "STOP":
            logger.info("Received STOP event")
            self.stop()

def main():
    """TODO: Add docstring."""
    parser = argparse.ArgumentParser(description="Dummy input generator for dora-gr00t")
    parser.add_argument("--interval", type=float, default=1.0, 
                        help="Interval between inputs (in seconds)")
    args = parser.parse_args()
    
    try:
        # Create the Dora node
        node = Node()
        logger.info("Created Dora node")
        
        # Initialize the dummy input node
        dummy_node = DummyInputNode(node, interval=args.interval)
        
        # Register event handler
        node.on_event(dummy_node.process_event)
        
        # Start the node and input generation
        logger.info("Starting Dora node")
        dummy_node.start()
        
        # Start the node loop
        for event in node:
            if event["type"] == "INPUT":
                logger.info(f"Received input event: {event}")
            elif event["type"] == "OUTPUT":
                logger.info(f"Received output event: {event}")
            # Other events are handled by the process_event callback
            
    except Exception as e:
        logger.error(f"Error in main: {e}")
        raise

if __name__ == "__main__":
    main()
