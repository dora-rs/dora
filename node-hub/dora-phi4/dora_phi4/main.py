import pyarrow as pa
import torch
from accelerate import infer_auto_device_map
from dora import Node
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
    MODEL_PATH, trust_remote_code=True, use_fast=True,
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
    MODEL_PATH, **MODEL_CONFIG, device_map=device_map,
)

generation_config = GenerationConfig.from_pretrained(MODEL_PATH)

user_prompt = "<|user|>"
assistant_prompt = "<|assistant|>"
prompt_suffix = "<|end|>"


def main():
    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            input_id = event["id"]
            if input_id == "text":
                text = event["value"][0].as_py()
                prompt = f"{user_prompt}{text}{prompt_suffix}{assistant_prompt}"

                # Process input
                inputs = processor(
                    text=prompt,
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
                node.send_output("text", pa.array([response]))


if __name__ == "__main__":
    main()
