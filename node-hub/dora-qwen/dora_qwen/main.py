import os

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
    """Load a GGUF model using llama-cpp-python with GPU/CPU fallback"""
    if VERBOSE:
        print(f"Loading model from {MODEL_REPO_ID}")
    
    use_gpu = os.getenv('USE_GPU', 'auto').lower()
    n_gpu_layers = {
        'auto': -1,
        'cpu': 0,
        'gpu': -1
    }.get(use_gpu, -1)
    
    try:
        llm = Llama.from_pretrained(
            repo_id=MODEL_REPO_ID,
            filename=MODEL_FILENAME,
            verbose=VERBOSE,
            n_ctx=2048,
            n_gpu_layers=n_gpu_layers
        )
    except RuntimeError as e:
        if 'cuda' in str(e).lower() and n_gpu_layers != 0:
            if VERBOSE:
                print('GPU loading failed, falling back to CPU')
            # Retry with CPU
            llm = Llama.from_pretrained(
                repo_id=MODEL_REPO_ID,
                filename=MODEL_FILENAME,
                verbose=VERBOSE,
                n_ctx=2048,
                n_gpu_layers=0
            )
    
    if VERBOSE:
        print("Model loaded successfully")
    
    return llm

def format_prompt(text: str) -> str:
    """Format prompt using Qwen chat template.
    
    Args:
        text: User input text to be formatted
        
    Returns:
        Formatted prompt string using Qwen's special tokens and template
    """
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