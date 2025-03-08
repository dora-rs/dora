import os
import sys
from typing import Dict, List, Optional, Tuple, Any, Union

import pyarrow as pa
from dora import Node
from llama_cpp import Llama

# Configuration via environment variables with defaults
MODEL_REPO_ID = os.getenv("MODEL_REPO_ID", "Qwen/Qwen2.5-0.5B-Instruct-GGUF")
MODEL_FILENAME = os.getenv("MODEL_FILENAME", "*fp16.gguf")
SYSTEM_PROMPT = os.getenv("SYSTEM_PROMPT", "You're a very succinct AI assistant with short answers.")
MAX_TOKENS = int(os.getenv("MAX_TOKENS", "256"))
TEMPERATURE = float(os.getenv("TEMPERATURE", "0.7"))
ACTIVATION_WORDS = os.getenv("ACTIVATION_WORDS", "what how who where you").split()
VERBOSE = os.getenv("VERBOSE", "false").lower() == "true"

def get_model() -> Llama:
    """Load a GGUF model using llama-cpp-python"""
    if VERBOSE:
        print(f"Loading model from {MODEL_REPO_ID}")
    
    llm = Llama.from_pretrained(
        repo_id=MODEL_REPO_ID,
        filename=MODEL_FILENAME,
        verbose=VERBOSE,
        n_ctx=2048,
        n_gpu_layers=-1  # Auto-detect GPU layers
    )
    
    if VERBOSE:
        print(f"Model loaded successfully")
    
    return llm

def format_prompt(text: str) -> str:
    """Format prompt using Qwen chat template"""
    messages = [
        {"role": "system", "content": SYSTEM_PROMPT},
        {"role": "user", "content": text}
    ]
    # Qwen chat template format
    prompt = "<|im_start|>system\n{system_message}<|im_end|>\n<|im_start|>user\n{user_message}<|im_end|>\n<|im_start|>assistant\n"
    return prompt.format(
        system_message=SYSTEM_PROMPT,
        user_message=text
    )

def main() -> None:
    """Main function that processes text inputs through the LLM."""
    try:
        model = get_model()
        if VERBOSE:
            print(f"Model initialized with system prompt: {SYSTEM_PROMPT}")
    except Exception as e:
        print(f"Error loading model: {e}")
        raise

    node = Node()
    
    for event in node:
        if event["type"] == "INPUT":
            try:
                text = event["value"][0].as_py()
                words = text.lower().split()
                
                if any(word in ACTIVATION_WORDS for word in words):
                    if VERBOSE:
                        print(f"Processing input: {text}")
                    
                    # Format prompt using chat template
                    prompt = format_prompt(text)
                    
                    # Generate response with increased context
                    response = model(
                        prompt=prompt,
                        max_tokens=MAX_TOKENS,
                        temperature=TEMPERATURE,
                        stop=["<|im_end|>", "<|im_start|>"]  # Use Qwen special tokens
                    )["choices"][0]["text"].strip()
                    
                    if VERBOSE:
                        print(f"Generated response: {response}")
                    
                    node.send_output(
                        output_id="text", 
                        data=pa.array([response]), 
                        metadata={}
                    )
            except Exception as e:
                print(f"Error processing input: {e}")
                node.send_output(
                    output_id="error",
                    data=pa.array([str(e)]),
                    metadata={}
                )

if __name__ == "__main__":
    main()