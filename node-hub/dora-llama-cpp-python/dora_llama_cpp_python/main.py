import os
import pyarrow as pa
from dora import Node
from transformers import AutoModelForCausalLM, AutoTokenizer

# Environment variables for model configuration
SYSTEM_PROMPT = os.getenv(
    "SYSTEM_PROMPT",
    "You're a very succinct AI assistant with short answers.",
)
MODEL_BACKEND = os.getenv("MODEL_BACKEND", "llama-cpp")
MODEL_REPO_ID = os.getenv("MODEL_REPO_ID", "Qwen/Qwen2.5-0.5B-Instruct-GGUF")
MODEL_FILENAME = os.getenv("MODEL_FILENAME", "*fp16.gguf")
HF_MODEL_NAME = os.getenv("HF_MODEL_NAME", "Qwen/Qwen2.5-0.5B-Instruct")
MAX_TOKENS = int(os.getenv("MAX_TOKENS", "512"))


def get_model_llama_cpp():
    """Load a GGUF model using llama-cpp-python (CPU by default)."""
    from llama_cpp import Llama

    llm = Llama.from_pretrained(
        repo_id=MODEL_REPO_ID,
        filename=MODEL_FILENAME,
        verbose=False
    )
    return llm


def get_model_huggingface():
    """Load a Hugging Face transformers model."""
    model = AutoModelForCausalLM.from_pretrained(
        HF_MODEL_NAME,
        torch_dtype="auto",
        device_map="cpu"
    )
    tokenizer = AutoTokenizer.from_pretrained(HF_MODEL_NAME)
    return model, tokenizer


ACTIVATION_WORDS = os.getenv("ACTIVATION_WORDS", "what how who where you").split()


def generate_hf(model, tokenizer, prompt: str, history) -> str:
    """Generates text using a Hugging Face model."""
    history += [{"role": "user", "content": prompt}]
    text = tokenizer.apply_chat_template(
        history, tokenize=False, add_generation_prompt=True
    )
    model_inputs = tokenizer([text], return_tensors="pt").to("cpu")  
    generated_ids = model.generate(**model_inputs, max_new_tokens=MAX_TOKENS)
    generated_ids = [
        output_ids[len(input_ids):]
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
            # print(f"Input text: {text}")

            if any(word in ACTIVATION_WORDS for word in words):
                if MODEL_BACKEND == "llama-cpp":
                    response = model(
                        f"Q: {text} A: ",
                        max_tokens=MAX_TOKENS,
                        stop=["Q:", "\n"],
                    )["choices"][0]["text"]
                else:
                    response, history = generate_hf(model, tokenizer, text, history)
                
                # print(f"Generated response: {response}")
                node.send_output(
                    output_id="text", data=pa.array([response]), metadata={}
                )


if __name__ == "__main__":
    main()
