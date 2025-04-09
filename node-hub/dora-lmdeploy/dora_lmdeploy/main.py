"""TODO: Add docstring."""

import os

import cv2
import numpy as np
import pyarrow as pa
from dora import Node
from lmdeploy import TurbomindEngineConfig, pipeline
from PIL import Image

# Default model configuration
DEFAULT_MODEL = "internlm/internlm2-7b"
MODEL_NAME = os.getenv("MODEL_NAME", DEFAULT_MODEL)

# System prompt and default question
SYSTEM_PROMPT = os.getenv(
    "SYSTEM_PROMPT",
    "You're a very succinct AI assistant, that describes image with a very short sentence.",
)
DEFAULT_QUESTION = os.getenv(
    "DEFAULT_QUESTION",
    "Describe this image",
)

# Turbomind configuration
TURBOMIND_CACHE_DIR = os.getenv("TURBOMIND_CACHE_DIR", "./workspace")
TURBOMIND_TP = int(os.getenv("TURBOMIND_TP", "1"))
TURBOMIND_GPU_MEMORY_FRACTION = float(os.getenv("TURBOMIND_GPU_MEMORY_FRACTION", "0.8"))

# Generation parameters
MAX_LENGTH = int(os.getenv("MAX_LENGTH", "2048"))
TEMPERATURE = float(os.getenv("TEMPERATURE", "0.7"))
TOP_P = float(os.getenv("TOP_P", "0.9"))

# Initialize Turbomind engine config
engine_config = TurbomindEngineConfig(
    model_name=MODEL_NAME,
    tp=TURBOMIND_TP,
    cache_max_entry_count=0.8,
    gpu_memory_fraction=TURBOMIND_GPU_MEMORY_FRACTION,
    cache_dir=TURBOMIND_CACHE_DIR,
)

# Initialize pipeline
pipe = pipeline(
    model_path=MODEL_NAME,
    engine_config=engine_config,
    max_length=MAX_LENGTH,
    temperature=TEMPERATURE,
    top_p=TOP_P,
)


def process_image(storage, metadata):
    """Process image data from storage and metadata."""
    encoding = metadata["encoding"]
    width = metadata["width"]
    height = metadata["height"]

    if encoding in ["bgr8", "rgb8"] or encoding in ["jpeg", "jpg", "jpe", "bmp", "webp", "png"]:
        channels = 3
        storage_type = np.uint8
    else:
        raise RuntimeError(f"Unsupported image encoding: {encoding}")

    if encoding == "bgr8":
        frame = storage.to_numpy().astype(storage_type).reshape((height, width, channels))
        frame = frame[:, :, ::-1]  # BGR to RGB
    elif encoding == "rgb8":
        frame = storage.to_numpy().astype(storage_type).reshape((height, width, channels))
    elif encoding in ["jpeg", "jpg", "jpe", "bmp", "webp", "png"]:
        storage = storage.to_numpy()
        frame = cv2.imdecode(storage, cv2.IMREAD_COLOR)
        frame = frame[:, :, ::-1]  # BGR to RGB
    else:
        raise RuntimeError(f"Unsupported image encoding: {encoding}")

    return Image.fromarray(frame)


def generate_response(image, text, history=None):
    """Generate response using LMDeploy pipeline."""
    if history is None:
        history = []

    # Prepare the prompt
    if SYSTEM_PROMPT:
        history = [{"role": "system", "content": SYSTEM_PROMPT}] + history

    # Add the current interaction
    messages = history + [
        {
            "role": "user",
            "content": [
                {"type": "image", "image": image},
                {"type": "text", "text": text},
            ],
        },
    ]

    # Generate response using pipeline
    response = pipe(messages)
    return response.text, history + [{"role": "assistant", "content": response.text}]


def main():
    """TODO: Add docstring."""
    node = Node()
    history = []
    cached_text = DEFAULT_QUESTION
    current_image = None

    for event in node:
        event_type = event["type"]

        if event_type == "INPUT":
            event_id = event["id"]

            if "image" in event_id:
                # Process image input
                current_image = process_image(event["value"], event["metadata"])

            elif "text" in event_id:
                # Process text input
                if len(event["value"]) > 0:
                    text = event["value"][0].as_py()
                else:
                    text = cached_text

                cached_text = text

                if current_image is None:
                    continue

                # Generate response
                response, history = generate_response(current_image, text, history)

                # Send output
                node.send_output(
                    "text",
                    pa.array([response]),
                    {"image_id": event_id},
                )

        elif event_type == "ERROR":
            print("Event Error:" + event["error"])


if __name__ == "__main__":
    main()
