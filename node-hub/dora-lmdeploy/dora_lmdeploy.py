#!/usr/bin/env python3
import os
import io
import pyarrow as pa
import numpy as np
from PIL import Image
from dora import Node
from lmdeploy import pipeline

def get_image(value):
    try:
        # assume raw bytes (jpeg/png) for now as it's robust
        image_data = value.to_pybytes()
        image = Image.open(io.BytesIO(image_data))
        return image
    except Exception as e:
        print(f"Warning: Could not decode image: {e}", flush=True)
        return None

def main():
    model_path = os.getenv("MODEL_PATH", "Qwen/Qwen2.5-VL-7B-Instruct")
    backend = os.getenv("BACKEND", "turbomind")
    
    print(f"Initializing dora-lmdeploy with model: {model_path} (backend: {backend})", flush=True)
    
    try:
        # might take a while if downloading or loading onto GPU
        pipe = pipeline(model_path)
        print("Model loaded successfully.", flush=True)
    except Exception as e:
        print(f"Error loading model: {e}", flush=True)
        return

    node = Node()
    last_image = None
    
    print("Node started. Loop entering...", flush=True)

    for event in node:
        if event["type"] == "INPUT":
            event_id = event["id"]
            
            if event_id == "image":
                # store input image for later use
                img_val = event["value"]
                last_image = get_image(img_val)
                if last_image:
                    print("Image received and updated.", flush=True)
                    
            elif event_id == "text":
                text = event["value"][0].as_py()
                print(f"Received prompt: {text}", flush=True)

                if not text:
                    print("Ignoring empty text input.", flush=True)
                    continue
                
                try:
                    if last_image:
                        print("Running multimodal inference...", flush=True)
                        response = pipe((text, last_image))
                    else:
                        print("Running text inference...", flush=True)
                        response = pipe(text)
                    
                    output = response.text if hasattr(response, 'text') else str(response)
                    node.send_output("response", pa.array([output]))
                    print(f"Response sent.", flush=True)
                    
                except Exception as e:
                    print(f"Inference error: {e}", flush=True)
                    node.send_output("error", pa.array([str(e)]))

if __name__ == "__main__":
    main()
