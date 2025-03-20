"""TODO: Add docstring."""

import os
import re
import time

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

USE_FLASH_ATTN = False
try:
    import flash_attn as _  # noqa
    USE_FLASH_ATTN = True
except (ImportError, ModuleNotFoundError):
    pass

# Define model config
MODEL_CONFIG = {
    "torch_dtype": torch_dtype,
    "trust_remote_code": True,
    "_attn_implementation": "flash_attention_2"
    if device == "cuda" and USE_FLASH_ATTN
    else "eager",
    "low_cpu_mem_usage": True,
}

# Infer device map without full initialization
device_map = infer_auto_device_map(
    AutoModelForCausalLM.from_pretrained(MODEL_PATH, **MODEL_CONFIG),
)

# Load the model directly with the inferred device map
model = AutoModelForCausalLM.from_pretrained(MODEL_PATH, **MODEL_CONFIG).to(device)

generation_config = GenerationConfig.from_pretrained(MODEL_PATH)

user_prompt = "<|user|>"
assistant_prompt = "<|assistant|>"
prompt_suffix = "<|end|>"

LEAD_MODALITY = os.getenv("LEAD_MODALITY", "text")

BAD_SENTENCES = [
    "The stock market closed down by 0.1%.",
    "The stock market closed down by 0.1 percent.",
    "The stock market closed down by one.",
    "The market is closed on Monday and Tuesday.",
    "The market is closed on Mondays and Tuesdays.",
    "the first is the of the internet communicate people",
    "The first time I saw the movie, I was very impressed.",
    "The first one is the one that is the most important.",
    "The first one is the one that is the most common.",
    "The first time I saw the sea, I was very young.",
    "The first time I saw the sea was when I was a child.",
    "The sound of the wind is so loud.",
    "The first time I saw the sea.",
    "the first time saw the sea i was so happy"
    "The first time I saw the sea, I was very happy.",
    "The first time I saw the sea was in the movie.",
    "The first time I saw the movie was in the theater.",
    "The first time I saw the movie.",
    "the first i saw the video i was very impressed",
    "the first time saw the video i was like my god",
    "i am a student at the university of toronto",
    "I don't know what to do.",
    "translator elisabeth buffard reviewer denise rq",
    "Translator Elisabeth Buffard Reviewer Denise RQ.",
    "Translator Denise RQ Reviewer Denise RQ.",
    "the company also has a presence the united states canada brazil argentina mexico colombia peru chile uruguay",
    "the company also has a presence the united states canada brazil mexico argentina colombia chile peru and uruguay",
    "the company also has a presence the united states canada brazil argentina mexico and the united kingdom",
    "the company also presence in the united states canada brazil argentina mexico colombia peru chile uruguay",
    "The company also has a presence in the United States, Canada, Brazil, Argentina, Mexico, Colombia, Peru, Chile, and Uruguay."
    "the company also has presence in the united states canada the united kingdom",
    "company also has a presence the united states canada brazil argentina colombia mexico peru chile and uruguay",
    "the company also has a presence the united states canada and the united kingdom",
    "the company also announced that it will acquire the online retailer zappos",
    "the company has a market capitalization of 15 trillion yen",
]


def remove_text_noise(text: str, text_noise="") -> str:
    """Remove noise from text.

    Args:
    ----
        text (str): Original text
        text_noise (str): text to remove from the original text

    Returns:
    -------
        str: Cleaned text

    """
    # Handle the case where text_noise is empty
    if not text_noise.strip():
        return (
            text  # Return the original text if text_noise is empty or just whitespace
        )

    # Helper function to normalize text (remove punctuation, make lowercase, and handle hyphens)
    def normalize(s):
        # Replace hyphens with spaces to treat "Notre-Dame" and "notre dame" as equivalent
        s = re.sub(r"-", " ", s)
        # Remove other punctuation and convert to lowercase
        return re.sub(r"[^\w\s]", "", s).lower()

    # Normalize both text and text_noise
    normalized_text = normalize(text)
    normalized_noise = normalize(text_noise)

    # Split into words
    text_words = normalized_text.split()
    noise_words = normalized_noise.split()

    # Function to find and remove noise sequence flexibly
    def remove_flexible(text_list, noise_list):
        i = 0
        while i <= len(text_list) - len(noise_list):
            match = True
            extra_words = 0
            for j, noise_word in enumerate(noise_list):
                if i + j + extra_words >= len(text_list):
                    match = False
                    break
                # Allow skipping extra words in text_list
                while (
                    i + j + extra_words < len(text_list)
                    and text_list[i + j + extra_words] != noise_word
                ):
                    extra_words += 1
                    if i + j + extra_words >= len(text_list):
                        match = False
                        break
                if not match:
                    break
            if match:
                # Remove matched part
                del text_list[i : i + len(noise_list) + extra_words]
                i = max(0, i - len(noise_list))  # Adjust index after removal
            else:
                i += 1
        return text_list

    # Only remove parts of text_noise that are found in text
    cleaned_words = text_words[:]
    for noise_word in noise_words:
        if noise_word in cleaned_words:
            cleaned_words.remove(noise_word)

    # Reconstruct the cleaned text
    return " ".join(cleaned_words)


def main():
    """TODO: Add docstring."""
    node = Node()

    frames = {}
    image = None
    audios = None
    text = ""
    noise_timestamp = time.time()
    text_noise = ""

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
                if len(frames) == 1:
                    image_prompt = "<|image_1|>"
                elif len(frames) > 1:
                    raise ValueError("Multiple images are not supported yet!")
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

                # Remove noise filter after some time
                if time.time() - noise_timestamp > (
                    len(text_noise.split()) / 1.5
                ):  # WPS
                    text_noise = ""

                if (
                    response in BAD_SENTENCES
                    or "company also has a presence" in response
                    or "The first time I saw the" in response
                ):
                    continue
                ## Remove text noise independently of casing
                response = remove_text_noise(response, text_noise)
                if response.strip() == "" or response.strip() == ".":
                    continue
                node.send_output("text", pa.array([response]))
                noise_timestamp = time.time()
                text_noise = response


if __name__ == "__main__":
    main()
