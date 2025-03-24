"""Dora Node: MLX-based """

import logging
import os

import mlx.core as mx
import mlx.nn as nn
from dora import Node
from mlx_lm import load, generate
import pyarrow as pa

# Configure logging
logging.basicConfig(level=logging.INFO)

# Environment variables for model configuration
SYSTEM_PROMPT = os.getenv("SYSTEM_PROMPT", "")
MODEL_NAME = os.getenv("MODEL_NAME", "mlx-community/Qwen1.5-0.5B")
MAX_TOKENS = int(os.getenv("MAX_TOKENS", "512"))
HISTORY_ENABLED = os.getenv("HISTORY", "False").lower() == "true"

# Words that trigger the model to respond
ACTIVATION_WORDS = os.getenv("ACTIVATION_WORDS", "").split()


def load_model():
    """Load the MLX transformer model and tokenizer."""
    logging.info(f"Loading MLX model: {MODEL_NAME}")
    model, tokenizer = load(MODEL_NAME, dtype=mx.float16)
    logging.info("Model loaded successfully")
    return model, tokenizer


def generate_response(model, tokenizer, text: str, history) -> tuple[str, list]:
    """Generate response using the MLX model."""
    history.append({"role": "user", "content": text})

    prompt = tokenizer.apply_chat_template(history, tokenize=False, add_generation_prompt=True)

    response = generate(model, tokenizer, prompt, max_tokens=MAX_TOKENS)
    history.append({"role": "assistant", "content": response})

    return response, history


def main():
    """Run Dora Node for MLX chatbot."""
    # Load the model
    model, tokenizer = load_model()
    
    # Initialize chat history
    history = [{"role": "system", "content": SYSTEM_PROMPT}] if SYSTEM_PROMPT else []
    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            text = event["value"][0].as_py()
            words = text.lower().split()

            if not ACTIVATION_WORDS or any(word in ACTIVATION_WORDS for word in words):
                response, tmp_history = generate_response(model, tokenizer, text, history)
                history = tmp_history if HISTORY_ENABLED else history
                node.send_output(output_id="text", data=pa.array([response]), metadata={})


if __name__ == "__main__":
    main()            
