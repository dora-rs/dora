"""Dora node for generating text responses using a pre-trained language model, optimized for Apple M1, M2, M3 chips.

This node listens for input prompts on the 'text' channel, generates text using
a pre-trained model (default: SmolLM-135M-Instruct-4bit) optimized for Apple's M-series
chips via MLX, and sends responses to the 'text' output channel. The node can be configured
via environment variables and supports activation words to filter inputs.

Note: This node is only supported on macOS. It skips execution on Linux and Windows.
"""

import logging
import os
import platform
import sys
import time
from pathlib import Path

# Vérifier si la plateforme est macOS
if platform.system() != "Darwin":
    logging.basicConfig(level=logging.INFO)
    logging.info("mlx-lm is only supported on macOS. Skipping execution on %s.", platform.system())
    sys.exit(0)  # Sortir sans erreur pour éviter un échec CI

import pyarrow as pa
from dora import Node
from mlx_lm import load, generate

# Configure logging
logging.basicConfig(level=logging.INFO)

# Environment variables for model configuration
MODEL_PATH = os.getenv("MODEL_PATH", "mlx-community/SmolLM-135M-Instruct-4bit")
SYSTEM_PROMPT = os.getenv("SYSTEM_PROMPT", "")
MAX_TOKENS = int(os.getenv("MAX_TOKENS", "100"))
TEMPERATURE = float(os.getenv("TEMPERATURE", "0.7"))
CONTEXT_SIZE = int(os.getenv("CONTEXT_SIZE", "2048"))  # Context length for the model
ACTIVATION_WORDS = os.getenv("ACTIVATION_WORDS", "").split()

def get_model():
    """Load a pre-trained language model and tokenizer optimized for Apple M1/M2/M3 chips."""
    try:
        logging.info(f"Loading model from {MODEL_PATH} for Apple M-series optimization")
        model, tokenizer = load(
            MODEL_PATH, tokenizer_config={"eos_token": "<|im_end|>"}
        )
        logging.info("Model loaded successfully with MLX for M1/M2/M3 performance")
        return model, tokenizer
    except Exception as e:
        logging.exception(f"Error loading model: {e}")
        raise

def main():
    """Process input events and generate text responses using the loaded model.

    Optimized for Apple M1, M2, M3 chips using the MLX framework for efficient inference.
    Generates responses independently for each input, using only the system prompt as context.
    """
    # Initialize model and tokenizer
    model, tokenizer = get_model()
    node = Node()
    history = [{"role": "system", "content": SYSTEM_PROMPT}] if SYSTEM_PROMPT else []

    for event in node:
        if event["type"] == "INPUT" and event["id"] == "text":
            # Validate input
            if not isinstance(event["value"], pa.Array) or len(event["value"]) == 0:
                logging.error("Invalid input: expected a non-empty pyarrow.Array")
                continue
            text = event["value"][0].as_py()
            if not isinstance(text, str):
                logging.error("Invalid input: expected a string")
                continue

            words = text.lower().split()
            if len(ACTIVATION_WORDS) == 0 or any(
                word in ACTIVATION_WORDS for word in words
            ):
                try:
                    start_time = time.time()
                    messages = history + [{"role": "user", "content": text}]
                    formatted_prompt = tokenizer.apply_chat_template(
                        messages, add_generation_prompt=True
                    )

                    response = generate(
                        model,
                        tokenizer,
                        prompt=formatted_prompt,
                        max_tokens=MAX_TOKENS,
                        temp=TEMPERATURE,
                        verbose=False,
                    )

                    processing_time = time.time() - start_time
                    node.send_output(
                        output_id="text",
                        data=pa.array([response]),
                        metadata={
                            "processing_time": processing_time,
                            "model": MODEL_PATH,
                            "optimized_for": "Apple M1/M2/M3",
                        },
                    )

                except Exception as e:
                    logging.exception(f"Error generating response: {e}")

        elif event["type"] == "STOP":
            logging.info("Received STOP event, cleaning up...")
            model = None
            tokenizer = None
            break

if __name__ == "__main__":
    main()