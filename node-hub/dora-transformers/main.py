import os
import logging

import torch
from transformers import AutoTokenizer, AutoModelForCausalLM, pipeline
import pyarrow as pa
from dora import Node

# filepath: c:\Users\sande\OneDrive\Documents\GitHub\dora\node-hub\dora-transformers\main.py
def load_transformer_model():
    # Change the default model name to a valid one
    model_name = os.getenv("MODEL_NAME", "EleutherAI/gpt-neo-125M")
    device = "cuda" if torch.cuda.is_available() else "cpu"
    tokenizer = AutoTokenizer.from_pretrained(model_name)
    model = AutoModelForCausalLM.from_pretrained(model_name).to(device)
    return tokenizer, model, device

def main():
    logging.basicConfig(level=logging.INFO)
    tokenizer, model, device = load_transformer_model()
    # If using CUDA, device index 0 is used; otherwise, -1 uses CPU
    device_index = 0 if device == "cuda" else -1
    gen_pipeline = pipeline(
        "text-generation",
        model=model,
        tokenizer=tokenizer,
        device=device_index,
    )
    node = Node()
    logging.info("dora-transformers node started, waiting for text input...")
    for event in node:
        if event.get("type") == "INPUT" and event.get("id") == "text":
            # Expecting a text input event
            input_text = event["value"].to_pylist()[0]
            logging.info(f"Received input: {input_text}")
            # Generate output using the transformer model
            results = gen_pipeline(input_text, max_length=50)
            output_text = results[0]["generated_text"]
            logging.info(f"Generated output: {output_text}")
            node.send_output("text", pa.array([output_text]), event.get("metadata", {}))

if __name__ == "__main__":
    main()