"""Dora node implementation for LMDeploy-based inference.

This module provides a Dora node that uses LMDeploy's TurboMind engine for efficient
model inference on CUDA.
"""

import logging
import os

import cv2
import numpy as np
import pyarrow as pa
from dora import Node
from lmdeploy import ChatTemplateConfig, TurbomindEngineConfig, pipeline
from PIL import Image

# Configure logging
logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")

# Load environment variables with defaults
DEFAULT_PATH = "liuhaotian/llava-v1.5-7b"
MODEL_NAME_OR_PATH = os.getenv("MODEL_NAME_OR_PATH", DEFAULT_PATH)
SYSTEM_PROMPT = os.getenv(
    "SYSTEM_PROMPT",
    "You're a very succinct AI assistant that describes images with short sentences.",
)
DEFAULT_QUESTION = os.getenv(
    "DEFAULT_QUESTION",
    "Describe the image in a very short sentence.",
)
HISTORY_ENABLED = os.getenv("HISTORY", "False").lower() == "true"

# Initialize LmDeploy pipeline with TurboMind engine
try:
    pipe = pipeline(
        MODEL_NAME_OR_PATH,
        backend_config=TurbomindEngineConfig(
            session_len=8192,
            cache_max_entry_count=0.5,  # Adaptive cache size
            cache_block_seq_len=128,    # Efficient sequence caching
        ),
        chat_template_config=ChatTemplateConfig(model_name="vicuna"),
    )
    logging.info(f"Successfully initialized LMDeploy pipeline with model: {MODEL_NAME_OR_PATH}")
except Exception as e:
    logging.exception(f"Failed to initialize LMDeploy pipeline: {e}")
    raise

def process_image(event):
    """Extract and process image data from the event.
    
    Args:
        event (dict): Event containing image data and metadata.
        
    Returns:
        PIL.Image or None: Processed image if successful, None otherwise.

    """
    try:
        storage = event["value"].to_numpy()
        metadata = event.get("metadata", {})
        encoding = metadata.get("encoding", "")
        width = metadata.get("width")
        height = metadata.get("height")

        if width is None or height is None:
            logging.error("Missing width or height in image metadata.")
            return None

        if encoding in ["jpeg", "jpg", "png", "bmp", "webp"]:
            storage = np.frombuffer(storage, dtype=np.uint8)
            frame = cv2.imdecode(storage, cv2.IMREAD_COLOR)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        elif encoding == "rgb8":
            frame = storage.reshape((height, width, 3))
        elif encoding == "bgr8":
            frame = storage.reshape((height, width, 3))[:, :, ::-1]
        else:
            raise ValueError(f"Unsupported image encoding: {encoding}")

        return Image.fromarray(frame)
    except Exception as e:
        logging.exception(f"Error processing image: {e}")
        return None

def generate(frames: dict, question: str, history: list, image_id: str = None) -> tuple[str, list]:
    """Generate response using LMDeploy pipeline.
    
    Args:
        frames (dict): Dictionary of processed images.
        question (str): Input text/question.
        history (list): Chat history if enabled.
        image_id (str, optional): Specific image ID to process.
        
    Returns:
        tuple: (response text, updated history)

    """
    try:
        image = frames.get(image_id) if image_id else next(reversed(frames.values()), None)
        if not image:
            logging.warning("No valid image found for processing")
            return "No valid image available for processing.", history

        response = pipe([(question, [image])])
        response_text = response[0] if response else "Error: No response generated"

        if HISTORY_ENABLED:
            history.append((question, response_text))

        return response_text, history
    except Exception as e:
        logging.exception(f"Error during LMDeploy inference: {e}")
        return f"Error during inference: {e!s}", history

def main():
    """Main entry point for the Dora LMDeploy node."""
    pa.array([])  # initialize pyarrow array
    node = Node()
    frames = {}
    history = []
    cached_text = DEFAULT_QUESTION
    image_id = None

    for event in node:
        event_type = event.get("type")
        if not event_type:
            logging.warning("Received an event without a type.")
            continue

        event_id = event.get("id")
        if not event_id:
            logging.warning("Received an event without an ID.")
            continue

        if event_type == "INPUT":
            if "image" in event_id:
                image = process_image(event)
                if image:
                    frames[event_id] = image
                    logging.info(f"Successfully processed image: {event_id}")

            elif "text" in event_id:
                text = event.get("value")
                text = text[0].as_py() if text and len(text) > 0 else cached_text
                cached_text = text
                image_id = event.get("metadata", {}).get("image_id")

                if frames:
                    response, history = generate(frames, text, history, image_id)
                    node.send_output(
                        "text",
                        pa.array([response]),
                        {"image_id": image_id if image_id else "all"},
                    )
                    logging.info("Generated and sent response")
        else:
            logging.warning(f"Unhandled event type: {event_type}")

if __name__ == "__main__":
    main()
