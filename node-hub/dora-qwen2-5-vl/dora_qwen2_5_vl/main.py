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


def generate(
    frames: dict, texts: list[str], history, past_key_values=None, image_id=None
):
    """Generate the response to the question given the image using Qwen2 model."""
    if image_id is not None:
        images = [frames[image_id]]
    else:
        images = list(frames.values())

    messages = []

    # If the texts is string, convert it to a list
    if isinstance(texts, str):
        texts = [texts]

    for text in texts:
        if text.startswith("<|system|>\n"):
            messages.append(
                {
                    "role": "system",
                    "content": [
                        {"type": "text", "text": text.replace("<|system|>\n", "")},
                    ],
                }
            )
        elif text.startswith("<|assistant|>\n"):
            messages.append(
                {
                    "role": "assistant",
                    "content": [
                        {"type": "text", "text": text.replace("<|assistant|>\n", "")},
                    ],
                }
            )
        elif text.startswith("<|tool|>\n"):
            messages.append(
                {
                    "role": "tool",
                    "content": [
                        {"type": "text", "text": text.replace("<|tool|>\n", "")},
                    ],
                }
            )
        elif text.startswith("<|user|>\n<|im_start|>\n"):
            messages.append(
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "text",
                            "text": text.replace("<|user|>\n<|im_start|>\n", ""),
                        },
                    ],
                }
            )
        elif text.startswith("<|user|>\n<|vision_start|>\n"):
            # Handle the case where the text starts with <|user|>\n<|vision_start|>
            image_url = text.replace("<|user|>\n<|vision_start|>\n", "")

            # If the last message was from the user, append the image URL to it
            if messages[-1]["role"] == "user":
                messages[-1]["content"].append(
                    {
                        "type": "image",
                        "image": image_url,
                    }
                )
            else:
                messages.append(
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "image",
                                "image": image_url,
                            },
                        ],
                    }
                )
        else:
            messages.append(
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": text},
                    ],
                }
            )

    # If the last message was from the user, append the image URL to it
    if messages[-1]["role"] == "user":
        messages[-1]["content"] += [
            {
                "type": "image",
                "image": image,
                "resized_height": image.size[1] * IMAGE_RESIZE_RATIO,
                "resized_width": image.size[0] * IMAGE_RESIZE_RATIO,
            }
            for image in images
        ]
    else:
        messages.append(
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
                ],
            }
        )

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
        history = tmp_history + [
            {
                "role": "assistant",
                "content": [
                    {"type": "text", "text": output_text[0]},
                ],
            }
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
                    texts = event["value"].to_pylist()
                    image_id = event["metadata"].get("image_id", None)
                else:
                    texts = cached_text
                words = texts[-1].split()
                if len(ACTIVATION_WORDS) > 0 and all(
                    word not in ACTIVATION_WORDS for word in words
                ):
                    continue

                cached_text = texts

                # set the max number of tiles in `max_num`
                response, history, past_key_values = generate(
                    frames,
                    texts,
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
