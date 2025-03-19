"""TODO: Add docstring."""

import os
import sys

import pyarrow as pa
from dora import Node
from transformers import AutoModelForCausalLM, AutoTokenizer

SYSTEM_PROMPT = os.getenv(
    "SYSTEM_PROMPT",
    "You're a very succinct AI assistant with short answers.",
)


def get_model_gguf():
    """TODO: Add docstring."""
    from llama_cpp import Llama

    return Llama.from_pretrained(
        repo_id="Qwen/Qwen2.5-0.5B-Instruct-GGUF",
        filename="*fp16.gguf",
        verbose=False,
    )


def get_model_darwin():
    """TODO: Add docstring."""
    from mlx_lm import load

    model, tokenizer = load("mlx-community/Qwen2.5-0.5B-Instruct-8bit")
    return model, tokenizer


def get_model_huggingface():
    """TODO: Add docstring."""
    model_name = "Qwen/Qwen2.5-0.5B-Instruct"

    model = AutoModelForCausalLM.from_pretrained(
        model_name,
        torch_dtype="auto",
        device_map="auto",
    )
    tokenizer = AutoTokenizer.from_pretrained(model_name)
    return model, tokenizer


ACTIVATION_WORDS = os.getenv("ACTIVATION_WORDS", "").split()


def generate_hf(model, tokenizer, prompt: str, history) -> str:
    """TODO: Add docstring."""
    history += [{"role": "user", "content": prompt}]
    text = tokenizer.apply_chat_template(
        history,
        tokenize=False,
        add_generation_prompt=True,
    )
    model_inputs = tokenizer([text], return_tensors="pt").to(model.device)
    generated_ids = model.generate(**model_inputs, max_new_tokens=512)
    generated_ids = [
        output_ids[len(input_ids) :]
        for input_ids, output_ids in zip(model_inputs.input_ids, generated_ids)
    ]
    response = tokenizer.batch_decode(generated_ids, skip_special_tokens=True)[0]
    history += [{"role": "assistant", "content": response}]
    return response, history


def main():
    """TODO: Add docstring."""
    history = []
    # If OS is not Darwin, use Huggingface model
    if sys.platform == "darwin":
        model = get_model_gguf()
    elif sys.platform == "linux":
        model, tokenizer = get_model_huggingface()
    else:
        model, tokenizer = get_model_darwin()

    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            # Warning: Make sure to add my_output_id and my_input_id within the dataflow.
            text = event["value"][0].as_py()
            words = text.lower().split()

            if len(ACTIVATION_WORDS) == 0 or any(
                word in ACTIVATION_WORDS for word in words
            ):
                # On linux, Windows
                if sys.platform == "darwin":
                    response = model.create_chat_completion(
                        messages=[{"role": "user", "content": text}],  # Prompt
                        max_tokens=24,
                    )["choices"][0]["message"]["content"]
                elif sys.platform == "linux":
                    response, history = generate_hf(model, tokenizer, text, history)
                else:
                    from mlx_lm import generate

                    response = generate(
                        model,
                        tokenizer,
                        prompt=text,
                        verbose=False,
                        max_tokens=50,
                    )

                node.send_output(
                    output_id="text",
                    data=pa.array([response]),
                    metadata={},
                )


if __name__ == "__main__":
    main()
