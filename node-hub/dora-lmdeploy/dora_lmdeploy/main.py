"""Dora Node for LMDeploy-based inference."""

import os
import cv2
import numpy as np
import pyarrow as pa
from dora import Node
from PIL import Image
from lmdeploy import pipeline

# Environment variables
MODEL_NAME = os.getenv("MODEL_NAME", "internlm/internlm-xcomposer2-vl-7b")
DEFAULT_PROMPT = os.getenv("DEFAULT_PROMPT", "Describe this image.")
IMAGE_RESIZE_RATIO = float(os.getenv("IMAGE_RESIZE_RATIO", "1.0"))
ACTIVATION_WORDS = os.getenv("ACTIVATION_WORDS", "").split()

# Initialize LMDeploy pipeline
pipe = pipeline(model_name=MODEL_NAME, backend_config={"backend": "tensorrt"})


def generate(frames: dict, question: str, image_id=None):
    """Generate a response using LMDeploy given images and a question."""
    if image_id is not None:
        images = [frames[image_id]]
    else:
        images = list(frames.values())

    # Perform inference using LMDeploy pipeline
    result = pipe((question, images))
    return result.text.strip()


def main():
    """Main function to handle events in the Dora node."""
    pa.array([])  # Initialize PyArrow array
    node = Node()

    frames = {}
    cached_text = DEFAULT_PROMPT

    for event in node:
        event_type = event["type"]

        if event_type == "INPUT":
            event_id = event["id"]

            if "image" in event_id:
                # Process incoming image data
                storage = event["value"]
                metadata = event["metadata"]
                encoding = metadata["encoding"]
                width = metadata["width"]
                height = metadata["height"]

                if encoding in ["bgr8", "rgb8", "jpeg", "jpg", "jpe", "bmp", "webp", "png"]:
                    channels = 3
                    storage_type = np.uint8
                else:
                    raise RuntimeError(f"Unsupported image encoding: {encoding}")

                if encoding == "bgr8":
                    frame = (
                        storage.to_numpy()
                        .astype(storage_type)
                        .reshape((height, width, channels))
                    )
                    frame = frame[:, :, ::-1]  # Convert BGR to RGB
                elif encoding == "rgb8":
                    frame = (
                        storage.to_numpy()
                        .astype(storage_type)
                        .reshape((height, width, channels))
                    )
                elif encoding in ["jpeg", "jpg", "jpe", "bmp", "webp", "png"]:
                    storage = storage.to_numpy()
                    frame = cv2.imdecode(storage, cv2.IMREAD_COLOR)
                    frame = frame[:, :, ::-1]  # Convert BGR to RGB
                else:
                    raise RuntimeError(f"Unsupported image encoding: {encoding}")

                image = Image.fromarray(frame)
                frames[event_id] = image

            elif "text" in event_id:
                # Process incoming text data
                if len(event["value"]) > 0:
                    text = event["value"][0].as_py()
                    image_id = event["metadata"].get("image_id", None)
                else:
                    text = cached_text

                words = text.split()
                if len(ACTIVATION_WORDS) > 0 and all(word not in ACTIVATION_WORDS for word in words):
                    continue

                cached_text = text

                if len(frames.keys()) == 0:
                    continue

                # Generate response using LMDeploy pipeline
                response = generate(frames, text, image_id)
                node.send_output(
                    "text",
                    pa.array([response]),
                    {"image_id": image_id if image_id is not None else "all"},
                )

        elif event_type == "ERROR":
            print("Event Error:" + event["error"])


if __name__ == "__main__":
    main()
