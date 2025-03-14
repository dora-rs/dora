import io
from urllib.request import urlopen

import pyarrow as pa
import requests
import soundfile as sf
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
elif torch.backends.mps.is_available():
    device = "mps"
    torch_dtype = torch.float16  # Reduce memory usage for MPS
else:
    device = "cpu"
    torch_dtype = torch.bfloat16  # CPU uses bfloat16 for efficiency


# Load the model and processor
MODEL_PATH = "microsoft/Phi-4-multimodal-instruct"

processor = AutoProcessor.from_pretrained(MODEL_PATH, trust_remote_code=True)

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
device_map = infer_auto_device_map(AutoModelForCausalLM.from_pretrained(MODEL_PATH, **MODEL_CONFIG))

# Load the model directly with the inferred device map
model = AutoModelForCausalLM.from_pretrained(MODEL_PATH, **MODEL_CONFIG, device_map=device_map)

generation_config = GenerationConfig.from_pretrained(MODEL_PATH)

# Define prompt structure
USER_PROMPT = "<|user|>"
ASSISTANT_PROMPT = "<|assistant|>"
PROMPT_SUFFIX = "<|end|>"


def process_image(image_url):
    """Processes an image through the model and returns the response."""
    prompt = f"{USER_PROMPT}<|image_1|>What is shown in this image?{PROMPT_SUFFIX}{ASSISTANT_PROMPT}"

    # Download and open image
    image = Image.open(requests.get(image_url, stream=True).raw)

    # Process input
    inputs = processor(text=prompt, images=image, return_tensors="pt").to(model.device)

    # Generate response
    with torch.no_grad():
        generate_ids = model.generate(
            **inputs, max_new_tokens=512, generation_config=generation_config,
        )
        generate_ids = generate_ids[:, inputs["input_ids"].shape[1] :]

    response = processor.batch_decode(
        generate_ids, skip_special_tokens=True, clean_up_tokenization_spaces=False,
    )[0]
    return response


def process_audio(audio_url):
    """Processes an audio file through the model and returns the transcript + translation."""
    speech_prompt = "Transcribe the audio to text, and then translate the audio to French. Use <sep> as a separator."
    prompt = f"{USER_PROMPT}<|audio_1|>{speech_prompt}{PROMPT_SUFFIX}{ASSISTANT_PROMPT}"

    # Download and read audio file
    audio, samplerate = sf.read(io.BytesIO(urlopen(audio_url).read()))

    # Process input
    inputs = processor(
        text=prompt, audios=[(audio, samplerate)], return_tensors="pt",
    ).to(model.device)

    # Generate response
    with torch.no_grad():
        generate_ids = model.generate(
            **inputs, max_new_tokens=512, generation_config=generation_config,
        )
        generate_ids = generate_ids[:, inputs["input_ids"].shape[1] :]

    response = processor.batch_decode(
        generate_ids, skip_special_tokens=True, clean_up_tokenization_spaces=False,
    )[0]
    return response


def main():
    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            input_id = event["id"]
            value = event["value"]

            print(f"Received event: {input_id}, value: {value}")

            # Check if it's an image URL
            if input_id == "image_input":
                image_response = process_image(value.as_py())  # Convert from PyArrow
                node.send_output(
                    output_id="image_output", data=pa.array([image_response]),
                )

            # Check if it's an audio URL
            elif input_id == "audio_input":
                audio_response = process_audio(value.as_py())  # Convert from PyArrow
                node.send_output(
                    output_id="audio_output", data=pa.array([audio_response]),
                )


if __name__ == "__main__":
    main()
