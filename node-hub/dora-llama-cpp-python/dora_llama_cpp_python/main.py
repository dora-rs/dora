import os
import pyarrow as pa
from dora import Node
from pathlib import Path

# Environment variables for model configuration
SYSTEM_PROMPT = os.getenv(
    "SYSTEM_PROMPT",
    "You're a very succinct AI assistant with short answers.",
)
MODEL_PATH = os.getenv("MODEL_PATH", "./models/llama-2-7b-chat.Q4_K_M.gguf")
MAX_TOKENS = int(os.getenv("MAX_TOKENS", "512"))
N_GPU_LAYERS = int(os.getenv("N_GPU_LAYERS", "0"))  # Number of layers to offload to GPU
N_THREADS = int(os.getenv("N_THREADS", "4"))  # Number of CPU threads
CONTEXT_SIZE = int(os.getenv("CONTEXT_SIZE", "4096"))


def get_model():
    """Load a GGUF model using llama-cpp-python with optional GPU acceleration."""
    from llama_cpp import Llama
    
    model_path = Path(MODEL_PATH)
    if not model_path.exists():
        raise FileNotFoundError(
            f"Model file not found at {MODEL_PATH}. "
            "Download it using: wget -O models/llama-2-7b-chat.Q4_K_M.gguf "
            "https://huggingface.co/TheBloke/Llama-2-7B-Chat-GGUF/resolve/main/llama-2-7b-chat.Q4_K_M.gguf"
        )

    llm = Llama(
        model_path=str(model_path),
        n_gpu_layers=N_GPU_LAYERS,  # Enable GPU acceleration if > 0
        n_ctx=CONTEXT_SIZE,         # Maximum context size
        n_threads=N_THREADS,        # Control CPU threading
        verbose=False
    )
    return llm


ACTIVATION_WORDS = os.getenv("ACTIVATION_WORDS", "what how who where you").split()


def main():
    # Initialize model
    model = get_model()
    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            text = event["value"][0].as_py()
            words = text.lower().split()

            if any(word in ACTIVATION_WORDS for word in words):
                # Generate response using system prompt
                prompt = f"{SYSTEM_PROMPT}\nQ: {text}\nA:"
                response = model(
                    prompt,
                    max_tokens=MAX_TOKENS,
                    stop=["Q:", "\n"],
                )["choices"][0]["text"]
                
                node.send_output(
                    output_id="text", data=pa.array([response]), metadata={}
                )


if __name__ == "__main__":
    main()
