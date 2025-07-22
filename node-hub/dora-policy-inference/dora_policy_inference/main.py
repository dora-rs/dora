"""TODO."""

import os
import queue
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import pyarrow as pa
from dora import Node
from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.factory import get_policy_class
from lerobot.utils.control_utils import predict_action
from lerobot.utils.utils import get_safe_torch_device


class DoraPolicyInference:
    """Policy inference node for LeRobot policies."""

    def __init__(self):
        """Initialize the policy inference node."""
        self.message_queue = queue.Queue()

        self.data_buffer = {}
        self.cameras = self._parse_cameras()
        self.task_description = os.getenv("TASK_DESCRIPTION", "")

        self.policy = None
        self.inference_fps = int(os.getenv("INFERENCE_FPS", "30"))
        self.min_inference_interval = 1.0 / self.inference_fps
        self.last_inference_time = 0

        self.shutdown = False
        self._load_policy()

    def _parse_cameras(self) -> dict:
        """Parse camera configuration from environment variables."""
        camera_names_str = os.getenv("CAMERA_NAMES", "laptop,front")
        camera_names = [name.strip() for name in camera_names_str.split(",")]

        return camera_names

    def _load_policy(self):
        """Load the trained LeRobot policy."""
        model_path = Path(os.getenv("MODEL_PATH"))
        if not model_path:
            raise ValueError("MODEL_PATH environment variable must be set correctly.")

        config = PreTrainedConfig.from_pretrained(model_path)
        self.device = get_safe_torch_device(config.device)  # get device automatically
        config.device = self.device.type

        # Get the policy class and load pretrained weights
        policy_cls = get_policy_class(config.type)
        self.policy = policy_cls.from_pretrained(
            pretrained_name_or_path=model_path, config=config
        )

        self.policy.eval()  # Set to evaluation mode
        self.policy.to(self.device)

        self._output(f"Policy loaded successfully on device: {self.device}")
        self._output(f"Policy type: {config.type}")

    def _check_inference_trigger(self, input_id: str) -> bool:
        """Determine if inference should be triggered based on the input received."""
        current_time = time.time()
        # Assign lead camera as trigger for inference
        if input_id != self.cameras[0]:
            return False

        # don't run inference too frequently, in case camera fps is too high
        if current_time - self.last_inference_time < self.min_inference_interval:
            return False

        return True

    def _run_inference(self):
        """Run policy inference on current observations."""
        current_time = time.time()

        observation = {}
        for camera_name in self.cameras:
            if camera_name in self.data_buffer:
                image = self._convert_camera_data(
                    self.data_buffer[camera_name]["data"],
                    self.data_buffer[camera_name]["metadata"],
                )
                observation[f"observation.images.{camera_name}"] = image

        state = self._convert_robot_data(self.data_buffer["robot_state"]["data"])
        observation["observation.state"] = state

        action = (
            predict_action(
                observation=observation,
                policy=self.policy,
                device=self.device,
                use_amp=self.policy.config.use_amp,
                task=self.task_description,
            )
            .cpu()
            .numpy()
        )

        # Convert from degrees to radians
        action = np.deg2rad(action)
        self._send_action(action)

        self.last_inference_time = current_time

    def _convert_camera_data(self, dora_data, metadata) -> np.ndarray:
        """Convert camera data from Dora format to numpy."""
        height, width = metadata.get("height"), metadata.get("width")
        encoding = metadata.get("encoding", "rgb8")
        image = dora_data.to_numpy().reshape(height, width, 3)

        if encoding == "bgr8":
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        elif encoding == "yuv420":
            image = cv2.cvtColor(image, cv2.COLOR_YUV2RGB_I420)

        return image.astype(np.uint8)

    def _convert_robot_data(self, dora_data, convert_to_degrees=True) -> np.ndarray:
        """Convert robot joint data, LeRobot expects angles in degrees and float32."""
        joint_array = dora_data.to_numpy()
        if convert_to_degrees:
            joint_array = np.rad2deg(joint_array)
        return joint_array.astype(np.float32)

    def _send_action(self, action: np.ndarray):
        """Send action through message queue to be output by main thread."""
        action_message = {"action": action}
        self.message_queue.put(("action", action_message))

    def handle_input(self, input_id: str, data: Any, metadata: Any):
        """Handle incoming data and trigger inference if conditions are met."""
        # Update data buffer
        self.data_buffer[input_id] = {
            "data": data,
            "timestamp": time.time(),
            "metadata": metadata,
        }

        # Check if inference should be triggered
        if self._check_inference_trigger(input_id):
            self._run_inference()

    def get_pending_messages(self):
        """Get all pending messages from the queue."""
        messages = []
        while not self.message_queue.empty():
            messages.append(self.message_queue.get_nowait())

        return messages

    def _output(self, message: str):
        """Output status message."""
        self.message_queue.put(("status", message))
        print(message)

    def shutdown(self):
        """Shutdown the inference node."""
        self._output("Shutting down policy inference...")
        self.shutdown = True
        self._output("Policy inference shutdown complete")


def main():
    node = Node()
    inference = DoraPolicyInference()

    for event in node:
        messages = inference.get_pending_messages()
        for msg_type, msg_data in messages:
            if msg_type == "action":
                # Send robot action
                node.send_output(
                    output_id="robot_action",
                    data=pa.array(msg_data["action"]),
                    metadata={},
                )
            elif msg_type == "status":
                node.send_output(
                    output_id="status", data=pa.array([msg_data]), metadata={}
                )

        if event["type"] == "INPUT":
            inference.handle_input(
                event["id"], event["value"], event.get("metadata", {})
            )

    # inference.shutdown() # Not sure when to shutdown the node, and how to identify task completion


if __name__ == "__main__":
    main()
