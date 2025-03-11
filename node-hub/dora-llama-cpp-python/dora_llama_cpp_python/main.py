import os
import pyarrow as pa
from dora import Node
from transformers import AutoModelForCausalLM, AutoTokenizer

# System prompt
SYSTEM_PROMPT = os.getenv(
    "SYSTEM_PROMPT",
    "You're a very succinct AI assistant with short answers.",
)

# Model selection based on ENV variable
MODEL_BACKEND = os.getenv("MODEL_BACKEND", "llama-cpp")  # Default to CPU-based Llama


def get_model_llama_cpp():
    """Load a GGUF model using llama-cpp-python (CPU by default)."""
    from llama_cpp import Llama

    llm = Llama.from_pretrained(
        repo_id="Qwen/Qwen2.5-0.5B-Instruct-GGUF", filename="*fp16.gguf", verbose=False
    )
    return llm


def get_model_huggingface():
    """Load a Hugging Face transformers model."""
    model_name = "Qwen/Qwen2.5-0.5B-Instruct"

    model = AutoModelForCausalLM.from_pretrained(
        model_name, torch_dtype="auto", device_map="cpu"
    )
    tokenizer = AutoTokenizer.from_pretrained(model_name)
    return model, tokenizer


ACTIVATION_WORDS = os.getenv("ACTIVATION_WORDS", "what how who where you").split()


def generate_hf(model, tokenizer, prompt: str, history) -> str:
    """Generates text using a Hugging Face model."""
    history += [{"role": "user", "content": prompt}]
    text = tokenizer.apply_chat_template(
        history, tokenize=False, add_generation_prompt=True
    )
    model_inputs = tokenizer([text], return_tensors="pt").to("cpu")  
    generated_ids = model.generate(**model_inputs, max_new_tokens=512)
    generated_ids = [
        output_ids[len(input_ids) :]
        for input_ids, output_ids in zip(model_inputs.input_ids, generated_ids)
    ]
    response = tokenizer.batch_decode(generated_ids, skip_special_tokens=True)[0]
    history += [{"role": "assistant", "content": response}]
    return response, history


def main():
    history = []

    # Select model backend
    if MODEL_BACKEND == "llama-cpp":
        model = get_model_llama_cpp()
    else:
        model, tokenizer = get_model_huggingface()

    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            text = event["value"][0].as_py()
            words = text.lower().split()
            print(words)

            if any(word in ACTIVATION_WORDS for word in words):
                print("")
                if MODEL_BACKEND == "llama-cpp":
                    response = model(
                        f"Q: {text} A: ",
                        max_tokens=24,
                        stop=["Q:", "\n"],
                    )["choices"][0]["text"]
                else:
                    response, history = generate_hf(model, tokenizer, text, history)
                
                # log output 
                print(response)
                node.send_output(
                    output_id="text", data=pa.array([response]), metadata={}
                )


if __name__ == "__main__":
    main()
