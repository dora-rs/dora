"""TODO: Add docstring."""

import os
from pathlib import Path

import cv2
import numpy as np
import pyarrow as pa
from dora import Node
from PIL import Image
from qwen_vl_utils import process_vision_info
from transformers import AutoProcessor, Qwen2_5_VLForConditionalGeneration

DEFAULT_PATH = "Qwen/Qwen2.5-VL-3B-Instruct"

MODEL_NAME_OR_PATH = os.getenv("MODEL_NAME_OR_PATH", DEFAULT_PATH)

if bool(os.getenv("USE_MODELSCOPE_HUB") in ["True", "true"]):
    from modelscope import snapshot_download

    if not Path(MODEL_NAME_OR_PATH).exists():
        MODEL_NAME_OR_PATH = snapshot_download(MODEL_NAME_OR_PATH)

SYSTEM_PROMPT = os.getenv(
    "SYSTEM_PROMPT",
    "You're a very succinct AI assistant, that describes image with a very short sentence.",
)
ACTIVATION_WORDS = os.getenv("ACTIVATION_WORDS", "").split()

DEFAULT_QUESTION = os.getenv(
    "DEFAULT_QUESTION",
    "Describe this image",
)
IMAGE_RESIZE_RATIO = float(os.getenv("IMAGE_RESIZE_RATIO", "1.0"))
HISTORY = os.getenv("HISTORY", "False") in ["True", "true"]
ADAPTER_PATH = os.getenv("ADAPTER_PATH", "")


# Check if flash_attn is installed
try:
    import flash_attn as _  # noqa

    model = Qwen2_5_VLForConditionalGeneration.from_pretrained(
        MODEL_NAME_OR_PATH,
        torch_dtype="auto",
        device_map="auto",
        attn_implementation="flash_attention_2",
    )
except (ImportError, ModuleNotFoundError):
    model = Qwen2_5_VLForConditionalGeneration.from_pretrained(
        MODEL_NAME_OR_PATH,
        torch_dtype="auto",
        device_map="auto",
    )


if ADAPTER_PATH != "":
    model.load_adapter(ADAPTER_PATH, "dora")


# default processor
processor = AutoProcessor.from_pretrained(MODEL_NAME_OR_PATH, use_fast=True)


def generate(frames: dict, question, history, past_key_values=None, image_id=None):
    """Generate the response to the question given the image using Qwen2 model."""
    if image_id is not None:
        images = [frames[image_id]]
    else:
        images = list(frames.values())
    messages = [
        {
            "role": "user",
            "content": [
                {
                    "type": "image",
                    "image": image,
                    "resized_height": image.size[1] * IMAGE_RESIZE_RATIO,
                    "resized_width": image.size[0] * IMAGE_RESIZE_RATIO,
                }
                for image in images
            ]
            + [
                {"type": "text", "text": question},
            ],
        },
    ]
    tmp_history = history + messages
    # Preparation for inference
    text = processor.apply_chat_template(
        tmp_history,
        tokenize=False,
        add_generation_prompt=True,
    )
    image_inputs, video_inputs = process_vision_info(messages)
    inputs = processor(
        text=[text],
        images=image_inputs,
        videos=video_inputs,
        padding=True,
        return_tensors="pt",
    )

    inputs = inputs.to(model.device)

    # Inference: Generation of the output
    ## TODO: Add past_key_values to the inputs when https://github.com/huggingface/transformers/issues/34678 is fixed.
    outputs = model.generate(
        **inputs,
        max_new_tokens=128,  # past_key_values=past_key_values
    )
    # past_key_values = outputs.past_key_values

    generated_ids_trimmed = [
        out_ids[len(in_ids) :] for in_ids, out_ids in zip(inputs.input_ids, outputs)
    ]
    output_text = processor.batch_decode(
        generated_ids_trimmed,
        skip_special_tokens=True,
        clean_up_tokenization_spaces=False,
    )
    if HISTORY:
        history += [
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": question},
                ],
            },
            {
                "role": "assistant",
                "content": [
                    {"type": "text", "text": output_text[0]},
                ],
            },
        ]

    return output_text[0], history, past_key_values


def main():
    """TODO: Add docstring."""
    pa.array([])  # initialize pyarrow array
    node = Node()

    if SYSTEM_PROMPT:
        history = [
            {
                "role": "system",
                "content": [
                    {"type": "text", "text": SYSTEM_PROMPT},
                ],
            },
        ]
    else:
        history = []

    cached_text = DEFAULT_QUESTION
    frames = {}
    image_id = None
    past_key_values = None

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
                        storage.to_numpy()
                        .astype(storage_type)
                        .reshape((height, width, channels))
                    )
                    frame = frame[:, :, ::-1]  # OpenCV image (BGR to RGB)
                elif encoding == "rgb8":
                    frame = (
                        storage.to_numpy()
                        .astype(storage_type)
                        .reshape((height, width, channels))
                    )
                elif encoding in ["jpeg", "jpg", "jpe", "bmp", "webp", "png"]:
                    storage = storage.to_numpy()
                    frame = cv2.imdecode(storage, cv2.IMREAD_COLOR)
                    frame = frame[:, :, ::-1]  # OpenCV image (BGR to RGB)
                else:
                    raise RuntimeError(f"Unsupported image encoding: {encoding}")
                image = Image.fromarray(frame)
                frames[event_id] = image

            elif "text" in event_id:
                if len(event["value"]) > 0:
                    text = event["value"][0].as_py()
                    image_id = event["metadata"].get("image_id", None)
                else:
                    text = cached_text
                words = text.split()
                if len(ACTIVATION_WORDS) > 0 and all(
                    word not in ACTIVATION_WORDS for word in words
                ):
                    continue

                cached_text = text

                if len(frames.keys()) == 0:
                    continue
                # set the max number of tiles in `max_num`
                response, history, past_key_values = generate(
                    frames,
                    text,
                    history,
                    past_key_values,
                    image_id,
                )
                metadata = event["metadata"]
                metadata["image_id"] = image_id if image_id is not None else "all"
                node.send_output(
                    "text",
                    pa.array([response]),
                    metadata,
                )

        elif event_type == "ERROR":
            print("Event Error:" + event["error"])


if __name__ == "__main__":
    main()
