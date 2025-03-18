"""TODO: Add docstring."""

import os

import cv2
import numpy as np
import pyarrow as pa
import torch
from accelerate import infer_auto_device_map
from dora import Node
from PIL import Image
from transformers import (
    AutoModelForCausalLM,
    AutoProcessor,
    GenerationConfig,
)

# 🔍 Detect the best available device
if torch.cuda.is_available():
    device = "cuda"
    torch_dtype = torch.float16  # Use float16 for efficiency
# TODO: Uncomment this once phi4 support mps backend.
# elif torch.backends.mps.is_available():
# device = "mps"
# torch_dtype = torch.float16  # Reduce memory usage for MPS
else:
    device = "cpu"
    torch_dtype = torch.bfloat16  # CPU uses bfloat16 for efficiency


# Load the model and processor
MODEL_PATH = "microsoft/Phi-4-multimodal-instruct"

processor = AutoProcessor.from_pretrained(
    MODEL_PATH,
    trust_remote_code=True,
    use_fast=True,
)

# Define model config
MODEL_CONFIG = {
    "torch_dtype": torch_dtype,
    "trust_remote_code": True,
    "_attn_implementation": "flash_attention_2"
    if device == "cuda" and torch.cuda.get_device_properties(0).total_memory > 16e9
    else "eager",
    "low_cpu_mem_usage": True,
}

# Infer device map without full initialization
device_map = infer_auto_device_map(
    AutoModelForCausalLM.from_pretrained(MODEL_PATH, **MODEL_CONFIG),
)

# Load the model directly with the inferred device map
model = AutoModelForCausalLM.from_pretrained(
    MODEL_PATH, **MODEL_CONFIG, device_map=device_map
).to(device)

generation_config = GenerationConfig.from_pretrained(MODEL_PATH)

user_prompt = "<|user|>"
assistant_prompt = "<|assistant|>"
prompt_suffix = "<|end|>"

LEAD_MODALITY = os.getenv("LEAD_MODALITY", "text")

BAD_SENTENCES = [
    "The stock market closed down by 0.1%.",
    "The stock market closed down by 0.1 percent.",
    "The market is closed on Mondays and Tuesdays.",
    "The first time I saw the movie, I was very impressed.",
    "The first time I saw the sea, I was very young.",
    "The first time I saw the sea was when I was a child.",
    "The sound of the wind is so loud.",
    "The first time I saw the sea.",
    "The first time I saw the sea was in the movie.",
    "The first time I saw the movie.",
    "I don't know what to do.",
    "I don't know.",
]


def main():
    """TODO: Add docstring."""
    node = Node()

    frames = {}
    image_id = None
    image = None
    audios = None
    text = ""
    for event in node:
        if event["type"] == "INPUT":
            input_id = event["id"]

            if "image" in input_id:
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
                frames[input_id] = image
            elif input_id == "audio":
                audio = event["value"].to_numpy()
                sample_rate = event["metadata"]["sample_rate"]
                audios = [(audio, sample_rate)]
            elif input_id == "text":
                text = event["value"][0].as_py()

            if LEAD_MODALITY == input_id:
                if len(frames) > 1:
                    raise ValueError("Multiple images are not supported yet!")
                elif len(frames) == 1:
                    image_prompt = "<|image_1|>"
                else:
                    image_prompt = ""

                if audios is not None:
                    audio_prompt = "<|audio_1|>"
                else:
                    audio_prompt = ""

                prompt = f"{user_prompt}{audio_prompt}{image_prompt}{text}{prompt_suffix}{assistant_prompt}"

                # Process input
                inputs = processor(
                    text=prompt,
                    images=image,
                    audios=audios,
                    return_tensors="pt",
                ).to(model.device)
                # Generate response
                with torch.no_grad():
                    generate_ids = model.generate(
                        **inputs,
                        max_new_tokens=512,
                        generation_config=generation_config,
                    )
                    generate_ids = generate_ids[:, inputs["input_ids"].shape[1] :]

                response = processor.batch_decode(
                    generate_ids,
                    skip_special_tokens=True,
                    clean_up_tokenization_spaces=False,
                )[0]

                if response not in BAD_SENTENCES:
                    node.send_output("text", pa.array([response]))


if __name__ == "__main__":
    main()
