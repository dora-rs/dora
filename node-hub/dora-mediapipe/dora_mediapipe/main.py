"""TODO: Add docstring."""

import cv2
import mediapipe as mp
import numpy as np
import pyarrow as pa
from dora import Node

# Initialiser MediaPipe Pose
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()
mp_draw = mp.solutions.drawing_utils


def get_3d_coordinates(landmark, depth_frame, w, h, resolution, focal_length):
    """Convert 2D landmark coordinates to 3D coordinates."""
    cx, cy = int(landmark.x * w), int(landmark.y * h)
    if 0 < cx < w and 0 < cy < h:
        depth = depth_frame[cy, cx] / 1_000.0
        if depth > 0:
            fx, fy = focal_length
            ppx, ppy = resolution
            x = (cy - ppy) * depth / fy
            y = (cx - ppx) * depth / fx

            # Convert to right-handed coordinate system
            return [x, -y, depth]
    return [0, 0, 0]


def get_image(event: dict) -> np.ndarray:
    """Convert the image from the event to a numpy array.

    Args:
        event (dict): The event containing the image data.

    """
    storage = event["value"]
    metadata = event["metadata"]
    encoding = metadata["encoding"]
    width = metadata["width"]
    height = metadata["height"]

    if (
        encoding == "bgr8"
        or encoding == "rgb8"
        or encoding in ["jpeg", "jpg", "jpe", "bmp", "webp", "png"]
    ):
        channels = 3
        storage_type = np.uint8
    else:
        raise RuntimeError(f"Unsupported image encoding: {encoding}")

    if encoding == "bgr8":
        frame = (
            storage.to_numpy().astype(storage_type).reshape((height, width, channels))
        )
        frame = frame[:, :, ::-1]  # OpenCV image (BGR to RGB)
    elif encoding == "rgb8":
        frame = (
            storage.to_numpy().astype(storage_type).reshape((height, width, channels))
        )
    elif encoding in ["jpeg", "jpg", "jpe", "bmp", "webp", "png"]:
        storage = storage.to_numpy()
        frame = cv2.imdecode(storage, cv2.IMREAD_COLOR)
        frame = frame[:, :, ::-1]  # OpenCV image (BGR to RGB)
    else:
        raise RuntimeError(f"Unsupported image encoding: {encoding}")
    return frame


def main():
    """TODO: Add docstring."""
    node = Node()
    depth = None
    focal_length = None
    resolution = None

    for event in node:
        if event["type"] == "INPUT":
            event_id = event["id"]
            if "image" in event_id:
                rgb_image = get_image(event)
                width = rgb_image.shape[1]
                height = rgb_image.shape[0]
                pose_results = pose.process(rgb_image)
                if pose_results.pose_landmarks:
                    values = pose_results.pose_landmarks.landmark
                    values = np.array(
                        [
                            [landmark.x * width, landmark.y * height]
                            for landmark in pose_results.pose_landmarks.landmark
                        ]
                    )
                    # Warning: Make sure to add my_output_id and my_input_id within the dataflow.
                    node.send_output(
                        output_id="points2d",
                        data=pa.array(values.ravel()),
                        metadata={},
                    )
                    if depth is not None:
                        values = np.array(
                            [
                                get_3d_coordinates(
                                    landmark,
                                    depth,
                                    width,
                                    height,
                                    resolution,
                                    focal_length,
                                )
                                for landmark in pose_results.pose_landmarks.landmark
                            ]
                        )
                        # Warning: Make sure to add my_output_id and my_input_id within the dataflow.
                        node.send_output(
                            output_id="points3d",
                            data=pa.array(values.ravel()),
                            metadata={},
                        )

                else:
                    print("No pose landmarks detected.")
            elif "depth" in event_id:
                metadata = event["metadata"]
                _encoding = metadata["encoding"]
                width = metadata["width"]
                height = metadata["height"]
                focal_length = metadata["focal_length"]
                resolution = metadata["resolution"]

                depth = event["value"].to_numpy().reshape((height, width))


if __name__ == "__main__":
    main()
