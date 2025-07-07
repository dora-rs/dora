"""TODO: Add docstring."""

import os
import queue
import threading
import time
from typing import Any

import cv2
import numpy as np
import pyarrow as pa
from dora import Node
from lerobot.datasets.lerobot_dataset import LeRobotDataset


class DoraLeRobotRecorder:
    """Recorder class for LeRobot dataset."""

    def __init__(self):
        """Initialize the Recorder node."""
        self.message_queue = queue.Queue()

        self.dataset = None
        self.episode_active = False
        self.frame_count = 0

        self.data_buffer = {}
        self.buffer_lock = threading.Lock()

        self.episode_index = 0
        self.start_time = None
        self.cameras = self._get_cameras()
        self.total_episodes = int(
            os.getenv("TOTAL_EPISODES", "10")
        )  # Default to 10 episodes
        self.episode_duration = int(
            os.getenv("EPISODE_DURATION_S", "60")
        )  # Default to 60 seconds
        self.reset_duration = int(
            os.getenv("RESET_DURATION_S", "15")
        )  # Default to 15 seconds
        self.fps = int(os.getenv("FPS", "30"))  # Default to 30 FPS

        self.recording_started = False
        self.in_reset_phase = False
        self.last_episode = False
        self.reset_start_time = None

        self.frame_interval = 1.0 / self.fps
        self.last_frame_time = None
        self.shutdown = False

        self._setup_dataset()
        self._start_frame_timer()

    def _get_cameras(self) -> dict:
        """Get Camera config."""
        camera_names_str = os.getenv("CAMERA_NAMES")
        if camera_names_str:
            camera_names = [name.strip() for name in camera_names_str.split(",")]
        else:
            return {}

        cameras = {}
        for camera_name in camera_names:
            resolution = os.getenv(f"CAMERA_{camera_name.upper()}_RESOLUTION")
            if resolution:
                dims = [int(d.strip()) for d in resolution.split(",")]
                cameras[camera_name] = dims
            else:
                print(
                    f'Warning: Set CAMERA_{camera_name.upper()}_RESOLUTION: "height,width,channels"'
                )

        return cameras

    def _get_robot_joints(self) -> list:
        """Get robot joints."""
        joints_str = os.getenv("ROBOT_JOINTS")
        if joints_str:
            return [joint.strip() for joint in joints_str.split(",")]
        else:
            raise ValueError("ROBOT_JOINTS are not set.")

    def _get_tags(self) -> list:
        """Get tags for dataset."""
        tags_str = os.getenv("TAGS")
        if tags_str:
            return [tag.strip() for tag in tags_str.split(",")]
        return []

    def _setup_dataset(self):
        """Set up the LeRobot Dataset."""
        features = {}

        joint_names = self._get_robot_joints()
        features["action"] = {
            "dtype": "float32",
            "shape": (len(joint_names),),
            "names": joint_names,
        }
        features["observation.state"] = {
            "dtype": "float32",
            "shape": (len(joint_names),),
            "names": joint_names,
        }

        self.use_videos = os.getenv("USE_VIDEOS", "true").lower() == "true"
        for camera_name in self.cameras:
            features[f"observation.images.{camera_name}"] = {
                "dtype": "video" if self.use_videos else "image",
                "shape": self.cameras[camera_name],
                "names": ["height", "width", "channels"],
            }

        self.required_features = set(features.keys())

        features.update(
            {
                "timestamp": {"dtype": "float32", "shape": [1]},
                "frame_index": {"dtype": "int64", "shape": [1]},
                "episode_index": {"dtype": "int64", "shape": [1]},
                "index": {"dtype": "int64", "shape": [1]},
                "task_index": {"dtype": "int64", "shape": [1]},
            }
        )

        repo_id = os.getenv("REPO_ID", None)
        if repo_id is None:
            raise ValueError(
                "REPO_ID environment variable must be set to create dataset"
            )

        self.dataset = LeRobotDataset.create(
            repo_id=repo_id,
            fps=self.fps,
            features=features,
            root=os.getenv("ROOT_PATH", None),
            robot_type=os.getenv("ROBOT_TYPE", "your_robot_type"),
            use_videos=self.use_videos,
            image_writer_processes=int(os.getenv("IMAGE_WRITER_PROCESSES", "0")),
            image_writer_threads=int(os.getenv("IMAGE_WRITER_THREADS", "4"))
            * len(self.cameras),
        )

    def _check_episode_timing(self):
        """Check if we need to start/end Episodes."""
        current_time = time.time()

        if not self.recording_started:  # Start the first episode
            self._start_episode()
            self.recording_started = True
            return False

        # If in reset phase, check if reset time is over
        if self.in_reset_phase or self.last_episode:
            if current_time - self.reset_start_time >= self.reset_duration:
                self.in_reset_phase = False
                if self.episode_index < self.total_episodes:
                    self._start_episode()
                else:
                    self._output(f"All {self.total_episodes} episodes completed!")
                    return True
            return False

        # If episode is active, check if episode time is over
        if self.episode_active:
            if (current_time - self.start_time) >= self.episode_duration:
                self._end_episode()
                if self.episode_index < self.total_episodes:
                    self._start_reset_phase()
                else:
                    self.last_episode = True

        return False

    def _start_episode(self):
        """Start a new episode."""
        self.episode_active = True
        self.start_time = time.time()
        self.frame_count = 0
        self._output(f"Started episode {self.episode_index + 1}/{self.total_episodes}")

    def _end_episode(self):
        """End current episode and save to dataset."""
        self.episode_active = False
        if self.frame_count > 0:
            self._output(
                f"Saving episode index {self.episode_index} with {self.frame_count} frames"
            )
            self.dataset.save_episode()
            self.episode_index += 1
        else:
            self._output(f"Episode {self.episode_index} had no frames, skipping save")

    def _start_reset_phase(self):
        """Start the reset phase between episodes."""
        self.in_reset_phase = True
        self.reset_start_time = time.time()
        self._output(
            f"Reset phase started - {self.reset_duration}s break before next episode..."
        )

    def _start_frame_timer(self):
        """Start the frame timer thread."""
        self.stop_timer = False
        self.frame_timer_thread = threading.Thread(
            target=self._frame_timer_loop, daemon=True
        )
        self.frame_timer_thread.start()

    def _frame_timer_loop(self):
        """Frame timing loop."""
        while not self.stop_timer and not self.shutdown:
            current_time = time.time()

            if (
                self.episode_active
                and not self.in_reset_phase
                and (
                    self.last_frame_time is None
                    or current_time - self.last_frame_time >= self.frame_interval
                )
            ):
                self._add_frame()
                self.last_frame_time = current_time

            should_stop = self._check_episode_timing()
            if should_stop:
                self.stop_timer = True  # Signal to stop the timer
                break

            time.sleep(0.001)

    def handle_input(self, input_id: str, data: Any, metadata: Any):
        """Handle incoming data - Store the latest data."""
        # Only store data if not in reset phase
        if not self.in_reset_phase:
            with self.buffer_lock:
                self.data_buffer[input_id] = {
                    "data": data,
                    "timestamp": time.time(),
                    "metadata": metadata,
                }

        should_stop = self._check_episode_timing()
        if should_stop:
            self.shutdown = True

        return should_stop

    def _shutdown(self):
        """Shutdown the Recorder."""
        print("Shutting down recorder...")

        # Signal shutdown
        self.shutdown = True
        self.stop_timer = True

        if self.frame_timer_thread.is_alive():
            print("Waiting for frame timer thread to finish...")
            self.frame_timer_thread.join(timeout=5.0)
        else:
            print("Frame timer thread finished successfully")

        self.finalize_dataset()
        print("Recorder shutdown complete")

    def _convert_camera_data(self, dora_data, metadata) -> np.ndarray:
        """Convert camera data from 1D pyarrow array to numpy format."""
        height, width = metadata.get("height"), metadata.get("width")
        encoding = metadata.get("encoding")
        image = dora_data.to_numpy().reshape(height, width, 3)

        if encoding == "bgr8":
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        elif encoding == "yuv420":
            image = cv2.cvtColor(image, cv2.COLOR_YUV2RGB_I420)

        return image.astype(np.uint8)

    def convert_robot_data(self, dora_data, convert_degrees=True) -> np.ndarray:
        """Convert robot joint data, LeRobot expects angles in degrees and float32."""
        joint_array = dora_data.to_numpy()
        if convert_degrees:
            joint_array = np.rad2deg(joint_array)
        return joint_array.astype(np.float32)

    def _add_frame(self):
        """Add a frame to the dataset."""
        with self.buffer_lock:
            frame_data = {}
            ideal_timestamp = self.frame_count / self.fps

            for key, value in self.data_buffer.items():
                if key == "robot_action":
                    frame_data["action"] = self.convert_robot_data(
                        self.data_buffer["robot_action"]["data"]
                    )
                if key == "robot_state":
                    frame_data["observation.state"] = self.convert_robot_data(
                        self.data_buffer["robot_state"]["data"]
                    )
                if {"height", "width"} <= value.get("metadata", {}).keys():
                    camera_name = key
                    image = self._convert_camera_data(
                        self.data_buffer[camera_name]["data"],
                        self.data_buffer[camera_name]["metadata"],
                    )
                    frame_data[f"observation.images.{camera_name}"] = image

            missing_keys = self.required_features - set(
                frame_data.keys()
            )  # Ensure all required features are present
            if missing_keys:
                print(f"Missing required data in frame: {missing_keys}")
                return

            self.dataset.add_frame(
                frame=frame_data,
                task=os.getenv("SINGLE_TASK", "Your task"),
                timestamp=ideal_timestamp,
            )
            self.frame_count += 1

    def finalize_dataset(self):
        """Finalize dataset and optionally push to hub."""
        if self.episode_active:
            self._end_episode()

        if self.use_videos:
            self._output("Encoding videos...")
            self.dataset.encode_videos()

        if os.getenv("PUSH_TO_HUB", "false").lower() == "true":
            self._output("Pushing dataset to hub...")
            self.dataset.push_to_hub(
                tags=self._get_tags(),
                private=os.getenv("PRIVATE", "false").lower() == "true",
            )

        self._output(
            f"Dataset recording completed. Total episodes: {self.episode_index}"
        )

    def _output(self, message: str):
        """Output message."""
        # Put message in queue to send
        self.message_queue.put(message)
        print(message)

    def get_pending_messages(self):
        """Get all pending messages from the queue."""
        messages = []
        while not self.message_queue.empty():
            messages.append(self.message_queue.get_nowait())
        return messages


def main():
    node = Node()
    recorder = DoraLeRobotRecorder()

    print("Starting dataset recording")
    print(f"Total episodes: {recorder.total_episodes}")
    print(f"Episode duration: {recorder.episode_duration}s")
    print(f"Reset duration: {recorder.reset_duration}s")

    for event in node:
        pending_messages = recorder.get_pending_messages()
        for message in pending_messages:
            node.send_output(output_id="text", data=pa.array([message]), metadata={})

        if event["type"] == "INPUT":
            should_stop = recorder.handle_input(
                event["id"], event["value"], event.get("metadata", {})
            )
            if should_stop:
                print("All episodes completed, stopping recording...")
                break

    recorder._shutdown()


def train_main():
    from lerobot.scripts.train import train
    from lerobot.utils.utils import (
        init_logging,
    )

    init_logging()
    train()


if __name__ == "__main__":
    main()
