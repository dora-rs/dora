"""TODO: Add docstring."""

import logging
import os
from pathlib import Path

import pyarrow as pa
from dora import Node

# Configure logging
logging.basicConfig(level=logging.INFO)
# Environment variables for model configuration
SYSTEM_PROMPT = os.getenv(
    "SYSTEM_PROMPT",
    "",
)
MODEL_NAME_OR_PATH = os.getenv("MODEL_NAME_OR_PATH", "TheBloke/Llama-2-7B-Chat-GGUF")
MODEL_FILE_PATTERN = os.getenv("MODEL_FILE_PATTERN", "*Q4_K_M.gguf")
MAX_TOKENS = int(os.getenv("MAX_TOKENS", "512"))
N_GPU_LAYERS = int(os.getenv("N_GPU_LAYERS", "0"))
N_THREADS = int(os.getenv("N_THREADS", "4"))
CONTEXT_SIZE = int(os.getenv("CONTEXT_SIZE", "4096"))


def get_model():
    """Load a GGUF model using llama-cpp-python with optional GPU acceleration."""
    from llama_cpp import Llama

    try:
        # Check if path exists locally
        model_path = Path(MODEL_NAME_OR_PATH)
        if model_path.exists():
            logging.info(f"Loading local model from {MODEL_NAME_OR_PATH}")
            llm = Llama(
                model_path=str(model_path),
                n_gpu_layers=N_GPU_LAYERS,
                n_ctx=CONTEXT_SIZE,
                n_threads=N_THREADS,
                verbose=False,
            )
        else:
            # Load from HuggingFace
            logging.info(
                f"Downloading model {MODEL_NAME_OR_PATH} with pattern {MODEL_FILE_PATTERN}"
            )
            llm = Llama.from_pretrained(
                repo_id=MODEL_NAME_OR_PATH,
                filename=MODEL_FILE_PATTERN,
                n_gpu_layers=N_GPU_LAYERS,
                n_ctx=CONTEXT_SIZE,
                n_threads=N_THREADS,
                verbose=False,
            )

        logging.info("Model loaded successfully")
        return llm

    except Exception as e:
        logging.exception(f"Error loading model: {e}")
        raise


ACTIVATION_WORDS = os.getenv("ACTIVATION_WORDS", "").split()


def main():
    """TODO: Add docstring."""
    # Initialize model
    model = get_model()
    node = Node()
    history = [{"role": "system", "content": SYSTEM_PROMPT}] if SYSTEM_PROMPT else []
    for event in node:
        if event["type"] == "INPUT":
            text = event["value"][0].as_py()
            words = text.lower().split()

            if len(ACTIVATION_WORDS) == 0 or any(
                word in ACTIVATION_WORDS for word in words
            ):
                # Generate response using system prompt
                response = model.create_chat_completion(
                    messages=history
                    + [
                        {"role": "user", "content": text},
                    ],  # Prompt
                    max_tokens=MAX_TOKENS,
                )["choices"][0]["message"]["content"]

                node.send_output(
                    output_id="text",
                    data=pa.array([response]),
                    metadata={},
                )


if __name__ == "__main__":
    main()
