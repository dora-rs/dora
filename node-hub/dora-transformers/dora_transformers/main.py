"""TODO: Add docstring."""

import logging
import os

import pyarrow as pa
import torch
from dora import Node
from transformers import AutoModelForCausalLM, AutoTokenizer

# Configure logging
logging.basicConfig(level=logging.INFO)

# Environment variables for model configuration
SYSTEM_PROMPT = os.getenv(
    "SYSTEM_PROMPT",
    "",
)
MODEL_NAME = os.getenv("MODEL_NAME", "Qwen/Qwen2.5-0.5B-Instruct")
MAX_TOKENS = int(os.getenv("MAX_TOKENS", "512"))
DEVICE = os.getenv("DEVICE", "auto")
TORCH_DTYPE = os.getenv("TORCH_DTYPE", "auto")
HISTORY= os.getenv("HISTORY", "False").lower() == "true"


# Words that trigger the model to respond
ACTIVATION_WORDS = os.getenv("ACTIVATION_WORDS", "").split()


def load_model():
    """Load the transformer model and tokenizer."""
    logging.info(f"Loading model {MODEL_NAME} on {DEVICE}")

    # Memory efficient loading
    model_kwargs = {
        "torch_dtype": TORCH_DTYPE,
        "device_map": DEVICE,
    }

    model = AutoModelForCausalLM.from_pretrained(MODEL_NAME, **model_kwargs)
    tokenizer = AutoTokenizer.from_pretrained(MODEL_NAME)
    logging.info("Model loaded successfully")
    return model, tokenizer


def generate_response(model, tokenizer, text: str, history) -> tuple[str, list]:
    """Generate text using the transformer model."""
    history += [{"role": "user", "content": text}]

    prompt = tokenizer.apply_chat_template(
        history,
        tokenize=False,
        add_generation_prompt=True,
    )

    model_inputs = tokenizer([prompt], return_tensors="pt").to(model.device)

    with torch.inference_mode():
        generated_ids = model.generate(
            **model_inputs,
            max_new_tokens=MAX_TOKENS,
            pad_token_id=tokenizer.pad_token_id,
            repetition_penalty=1.2,
        )

    generated_ids = [
        output_ids[len(input_ids) :]
        for input_ids, output_ids in zip(model_inputs.input_ids, generated_ids)
    ]

    response = tokenizer.batch_decode(generated_ids, skip_special_tokens=True)[0]
    history += [{"role": "assistant", "content": response}]

    return response, history


def main():
    """TODO: Add docstring."""
    # Initialize model and conversation history
    model, tokenizer = load_model()
    # Initialize history with system prompt

    history = [{"role": "system", "content": SYSTEM_PROMPT}] if SYSTEM_PROMPT else []
    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            text = event["value"][0].as_py()
            words = text.lower().split()

            if len(ACTIVATION_WORDS) == 0 or any(
                word in ACTIVATION_WORDS for word in words
            ):
                response, tmp_history = generate_response(model, tokenizer, text, history)
                history = tmp_history if HISTORY else history
                node.send_output(
                    output_id="text", data=pa.array([response]), metadata={}
                )


if __name__ == "__main__":
    main()
