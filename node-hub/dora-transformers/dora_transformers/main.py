"""Dora Node for Transformer-based AI Inference.

This script sets up a Dora Node that listens for input events, processes text using a
pretrained transformer model, and generates AI responses. The model, tokenizer, and
execution parameters are configurable through environment variables.

Features:
- Uses PyTorch and Hugging Face transformers for text generation.
- Supports memory-efficient model loading on CUDA.
- Configurable model, token limits, and activation words via environment variables.

Environment Variables:
- SYSTEM_PROMPT: Default system message for the chatbot.
- MODEL_NAME: Hugging Face model name to use.
- MAX_TOKENS: Maximum tokens for response generation.
- DEVICE: Device for running the model (e.g., "cpu", "cuda", "auto").
- TORCH_DTYPE: Torch data type for model precision.
- ENABLE_MEMORY_EFFICIENT: Enables memory-efficient loading.
- ACTIVATION_WORDS: Words that trigger the model response.

"""

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
    "You're a very succinct AI assistant with short answers.",
)
MODEL_NAME = os.getenv("MODEL_NAME", "Qwen/Qwen2.5-0.5B-Instruct")
MAX_TOKENS = int(os.getenv("MAX_TOKENS", "512"))
DEVICE = os.getenv("DEVICE", "auto")
TORCH_DTYPE = os.getenv("TORCH_DTYPE", "auto")
ENABLE_MEMORY_EFFICIENT = os.getenv("ENABLE_MEMORY_EFFICIENT", "true").lower() == "true"

# Configure PyTorch memory management
if DEVICE == "cuda":
    os.environ["PYTORCH_CUDA_ALLOC_CONF"] = "expandable_segments:True"
    if ENABLE_MEMORY_EFFICIENT:
        torch.cuda.empty_cache()

# Words that trigger the model to respond
ACTIVATION_WORDS = os.getenv("ACTIVATION_WORDS", "what how who where you").split()

def load_model():
    """Load the transformer model and tokenizer."""
    logging.info(f"Loading model {MODEL_NAME} on {DEVICE}")

    # Memory efficient loading

def load_model():
    """Load the transformer model and tokenizer.

    This function loads the pretrained transformer model and tokenizer based on
    the specified MODEL_NAME. It applies memory-efficient settings if enabled.

    Returns
    -------
        tuple: A tuple containing the loaded model and tokenizer.

    """
    logging.info(f"Loading model {MODEL_NAME} on {DEVICE}")


def load_model():
    """Load the transformer model and tokenizer.

    This function loads the pretrained transformer model and tokenizer based on
    the specified MODEL_NAME. It applies memory-efficient settings if enabled.

    Returns
    -------
        tuple: A tuple containing the loaded model and tokenizer.

    """
    logging.info(f"Loading model {MODEL_NAME} on {DEVICE}")

    model_kwargs = {
        "torch_dtype": TORCH_DTYPE,
        "device_map": DEVICE,
    }

    if ENABLE_MEMORY_EFFICIENT and DEVICE == "cuda":
        model_kwargs.update({
            "low_cpu_mem_usage": True,
            "offload_folder": "offload",
            "load_in_8bit": True,
        })

    model = AutoModelForCausalLM.from_pretrained(
        MODEL_NAME,
        **model_kwargs,
    )
        model_kwargs.update(
            {
                "low_cpu_mem_usage": True,
                "offload_folder": "offload",
                "load_in_8bit": True,
            }
        )

    model = AutoModelForCausalLM.from_pretrained(MODEL_NAME, **model_kwargs)
    tokenizer = AutoTokenizer.from_pretrained(MODEL_NAME)
    logging.info("Model loaded successfully")
    return model, tokenizer


def generate_response(model, tokenizer, text: str, history) -> tuple[str, list]:
    """Generate a response using the transformer model.

    This function constructs a chatbot-style prompt, tokenizes the input, and
    generates text using the model. The response is added to the conversation history.

    Args:
    ----
        model: The transformer model used for text generation.
        tokenizer: The tokenizer for encoding/decoding text.
        text (str): The input text from the user.
        history (list): A list representing the chat history.

    Returns:
    -------
        tuple: A tuple containing the generated response and updated history.

    """
    history += [{"role": "user", "content": text}]

    prompt = tokenizer.apply_chat_template(
        history, tokenize=False, add_generation_prompt=True,
    )

    model_inputs = tokenizer([prompt], return_tensors="pt").to(DEVICE)

    with torch.inference_mode():
        generated_ids = model.generate(
            **model_inputs,
            max_new_tokens=MAX_TOKENS,
            pad_token_id=tokenizer.pad_token_id,
            do_sample=True,
            temperature=0.7,
            top_p=0.9,
            repetition_penalty=1.2,
            length_penalty=0.5,
        )

    generated_ids = [
        output_ids[len(input_ids) :]
        for input_ids, output_ids in zip(model_inputs.input_ids, generated_ids)
    ]

    response = tokenizer.batch_decode(generated_ids, skip_special_tokens=True)[0]
    history += [{"role": "assistant", "content": response}]

    # Clear CUDA cache after generation if memory efficiency is enabled
    if ENABLE_MEMORY_EFFICIENT and DEVICE == "cuda":
        torch.cuda.empty_cache()

    return response, history


def main():
    """Run the Dora Node for AI inference.

    This function initializes the model, tokenizer, and conversation history.
    It then creates a Dora Node to listen for incoming text inputs and generates
    AI responses when an activation word is detected.
    """
    model, tokenizer = load_model()
    history = [{"role": "system", "content": SYSTEM_PROMPT}]
    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            text = event["value"][0].as_py()
            words = text.lower().split()

            if any(word in ACTIVATION_WORDS for word in words):
                logging.info(f"Processing input: {text}")
                response, history = generate_response(model, tokenizer, text, history)
                logging.info(f"Generated response: {response}")

                node.send_output(
                    output_id="text", data=pa.array([response]), metadata={}
                )


if __name__ == "__main__":
    main()
