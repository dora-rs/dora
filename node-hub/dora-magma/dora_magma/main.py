"""TODO: Add docstring."""

import ast
import logging
import os
from pathlib import Path

import cv2
import numpy as np
import pyarrow as pa
import torch
from dora import Node
from PIL import Image
from transformers import AutoModelForCausalLM, AutoProcessor

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

current_dir = Path(__file__).parent.absolute()
magma_dir = current_dir.parent / "Magma" / "magma"


def load_magma_models():
    """TODO: Add docstring."""
    default_path = str(magma_dir.parent / "checkpoints" / "Magma-8B")
    if not os.path.exists(default_path):
        default_path = str(magma_dir.parent)
        if not os.path.exists(default_path):
            logger.warning(
                "Warning: Magma submodule not found, falling back to HuggingFace version",
            )
            default_path = "microsoft/Magma-8B"

    model_name_or_path = os.getenv("MODEL_NAME_OR_PATH", default_path)
    logger.info(f"Loading Magma model from: {model_name_or_path}")

    try:
        model = AutoModelForCausalLM.from_pretrained(
            model_name_or_path,
            trust_remote_code=True,
            torch_dtype=torch.bfloat16,
            device_map="auto",
        )
        processor = AutoProcessor.from_pretrained(
            model_name_or_path,
            trust_remote_code=True,
        )
    except Exception as e:
        logger.error(f"Failed to load model: {e}")
        raise

    return model, processor, model_name_or_path


model, processor, MODEL_NAME_OR_PATH = load_magma_models()


def generate(
    image: Image,
    text: str,
) -> tuple[str, dict]:
    """Generate text and trajectories for the given image and text."""
    conv_user = f"<image>\n{text}\n"
    if (
        hasattr(model.config, "mm_use_image_start_end")
        and model.config.mm_use_image_start_end
    ):
        conv_user = conv_user.replace("<image>", "<image_start><image><image_end>")

    convs = [
        {"role": "system", "content": "You are an agent that can see, talk, and act."},
        {"role": "user", "content": conv_user},
    ]

    prompt = processor.tokenizer.apply_chat_template(
        convs,
        tokenize=False,
        add_generation_prompt=True,
    )

    try:
        inputs = processor(images=image, texts=prompt, return_tensors="pt")
        inputs["pixel_values"] = inputs["pixel_values"].unsqueeze(0)
        inputs["image_sizes"] = inputs["image_sizes"].unsqueeze(0)
        inputs = inputs.to(model.device)

        with torch.inference_mode():
            output_ids = model.generate(
                **inputs,
                temperature=0.3,
                do_sample=True,
                num_beams=1,
                max_new_tokens=1024,
                use_cache=True,
            )
        response = processor.batch_decode(output_ids, skip_special_tokens=True)[0]

        # Parse trajectories from response
        trajectories = {}
        try:
            if "and their future positions are:" in response:
                _, traces_str = response.split("and their future positions are:\n")
            else:
                _, traces_str = None, response

            # Parse the trajectories using the same approach as in `https://github.com/microsoft/Magma/blob/main/agents/robot_traj/app.py`
            traces_dict = ast.literal_eval(
                "{" + traces_str.strip().replace("\n\n", ",") + "}",
            )
            for mark_id, trace in traces_dict.items():
                trajectories[mark_id] = ast.literal_eval(trace)
        except Exception as e:
            logger.warning(f"Failed to parse trajectories: {e}")

        return response, trajectories

    except Exception as e:
        logger.error(f"Error in generate: {e}")
        return f"Error: {e}", {}


def main():
    """TODO: Add docstring."""
    node = Node()
    frames = {}

    for event in node:
        event_type = event["type"]

        if event_type == "INPUT":
            event_id = event["id"]

            if "image" in event_id:
                storage = event["value"]
                metadata = event["metadata"]
                encoding = metadata["encoding"]
                width = metadata["width"]
                height = metadata["height"]

                try:
                    if encoding == "bgr8":
                        frame = (
                            storage.to_numpy()
                            .astype(np.uint8)
                            .reshape((height, width, 3))
                        )
                        frame = frame[:, :, ::-1]  # Convert BGR to RGB
                    elif encoding == "rgb8":
                        frame = (
                            storage.to_numpy()
                            .astype(np.uint8)
                            .reshape((height, width, 3))
                        )
                    elif encoding in ["jpeg", "jpg", "jpe", "bmp", "webp", "png"]:
                        storage = storage.to_numpy()
                        frame = cv2.imdecode(storage, cv2.IMREAD_COLOR)
                        if frame is None:
                            raise ValueError(
                                f"Failed to decode image with encoding {encoding}",
                            )
                        frame = frame[:, :, ::-1]  # Convert BGR to RGB
                    else:
                        raise ValueError(f"Unsupported image encoding: {encoding}")

                    image = Image.fromarray(frame)
                    frames[event_id] = image

                    # Cleanup old frames
                    if len(frames) > 10:
                        frames.popitem(last=False)
                except Exception as e:
                    logger.error(f"Error processing image {event_id}: {e}")

            # Handle text inputs
            elif "text" in event_id:
                if len(event["value"]) > 0:
                    task_description = event["value"][0].as_py()
                    image_id = event["metadata"].get("image_id", None)

                    if image_id in frames:
                        image = frames[image_id]
                    elif len(frames) == 1:
                        image = next(iter(frames.values()))
                    else:
                        logger.error(f"Image not found for {image_id}")
                        continue
                    response, trajectories = generate(image, task_description)
                    node.send_output(
                        "text",
                        pa.array([response]),
                        {"image_id": image_id},
                    )

                    # Send trajectory data if available
                    if trajectories:
                        import json

                        node.send_output(
                            "trajectories",
                            pa.array([json.dumps(trajectories)]),
                            {"image_id": image_id},
                        )
                else:
                    continue

        elif event_type == "ERROR":
            logger.error(f"Event Error: {event['error']}")


if __name__ == "__main__":
    main()
