import os
import pyarrow as pa
from dora import Node
from transformers import AutoModelForCausalLM, AutoTokenizer
import logging
import torch

# Configure logging
logging.basicConfig(level=logging.INFO)

# Environment variables for model configuration
SYSTEM_PROMPT = os.getenv(
    "SYSTEM_PROMPT",
    "You're a very succinct AI assistant with short answers.",
)
MODEL_NAME = os.getenv("MODEL_NAME", "Qwen/Qwen2.5-0.5B-Instruct")
MAX_TOKENS = int(os.getenv("MAX_TOKENS", "512"))
DEVICE = os.getenv("DEVICE", "cpu")
TORCH_DTYPE = os.getenv("TORCH_DTYPE", "auto")
ENABLE_MEMORY_EFFICIENT = os.getenv("ENABLE_MEMORY_EFFICIENT", "true").lower() == "true"

# Configure PyTorch memory management
if DEVICE == "cuda":
    # Set memory efficient settings
    os.environ["PYTORCH_CUDA_ALLOC_CONF"] = "expandable_segments:True"
    if ENABLE_MEMORY_EFFICIENT:
        torch.cuda.empty_cache()

# Words that trigger the model to respond
ACTIVATION_WORDS = os.getenv("ACTIVATION_WORDS", "what how who where you").split()

def load_model():
    """Load the transformer model and tokenizer."""
    try:
        logging.info(f"Loading model {MODEL_NAME} on {DEVICE}")
        
        # Memory efficient loading
        model_kwargs = {
            "torch_dtype": TORCH_DTYPE,
            "device_map": DEVICE,
        }
        
        if ENABLE_MEMORY_EFFICIENT and DEVICE == "cuda":
            model_kwargs.update({
                "low_cpu_mem_usage": True,
                "offload_folder": "offload",
                "load_in_8bit": True
            })
        
        model = AutoModelForCausalLM.from_pretrained(
            MODEL_NAME,
            **model_kwargs
        )
        tokenizer = AutoTokenizer.from_pretrained(MODEL_NAME)
        logging.info("Model loaded successfully")
        return model, tokenizer
    except Exception as e:
        if "CUDA out of memory" in str(e):
            logging.error("CUDA out of memory error. Try:")
            logging.error("1. Setting DEVICE=cpu")
            logging.error("2. Using a smaller model")
            logging.error("3. Setting ENABLE_MEMORY_EFFICIENT=true")
        logging.error(f"Error loading model: {e}")
        raise

def generate_response(model, tokenizer, text: str, history) -> tuple[str, list]:
    """Generate text using the transformer model."""
    try:
        history += [{"role": "user", "content": text}]
        prompt = tokenizer.apply_chat_template(
            history, tokenize=False, add_generation_prompt=True
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
                repetition_penalty=1.2,  # Reduce repetition
                length_penalty=0.5,
            )
        
        generated_ids = [
            output_ids[len(input_ids):]
            for input_ids, output_ids in zip(model_inputs.input_ids, generated_ids)
        ]
        
        response = tokenizer.batch_decode(generated_ids, skip_special_tokens=True)[0]
        history += [{"role": "assistant", "content": response}]
        
        # Clear CUDA cache after generation if enabled
        if ENABLE_MEMORY_EFFICIENT and DEVICE == "cuda":
            torch.cuda.empty_cache()
            
        return response, history
    except RuntimeError as e:
        if "CUDA out of memory" in str(e):
            logging.error("CUDA out of memory during generation. Falling back to CPU")
            model.to("cpu")
            return generate_response(model, tokenizer, text, history)
        raise

def main():
    # Initialize model and conversation history
    model, tokenizer = load_model()
    # Initialize history with system prompt
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
                    output_id="text", 
                    data=pa.array([response]), 
                    metadata={}
                )

if __name__ == "__main__":
    main()
