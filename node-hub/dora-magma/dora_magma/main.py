"""TODO: Add docstring."""  

import os
from pathlib import Path
import cv2
import numpy as np
import pyarrow as pa
import torch
from dora import Node
from PIL import Image
from transformers import AutoModelForCausalLM, AutoProcessor
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

current_dir = Path(__file__).parent.absolute()
magma_dir = current_dir.parent / "Magma" / "magma"

def load_magma_models():
    """TODO: Add docstring."""  
    DEFAULT_PATH = str(magma_dir.parent / "checkpoints" / "Magma-8B")
    if not os.path.exists(DEFAULT_PATH):
        DEFAULT_PATH = str(magma_dir.parent)
        if not os.path.exists(DEFAULT_PATH):
            logger.warning("Warning: Magma submodule not found, falling back to HuggingFace version")
            DEFAULT_PATH = "microsoft/Magma-8B"

    MODEL_NAME_OR_PATH = os.getenv("MODEL_NAME_OR_PATH", DEFAULT_PATH)
    logger.info(f"Loading Magma model from: {MODEL_NAME_OR_PATH}")
    
    try:
        model = AutoModelForCausalLM.from_pretrained(
            MODEL_NAME_OR_PATH, 
            trust_remote_code=True,
            torch_dtype=torch.bfloat16, 
            device_map="auto"
        )
        processor = AutoProcessor.from_pretrained(MODEL_NAME_OR_PATH, trust_remote_code=True)
    except Exception as e:
        logger.error(f"Failed to load model: {e}")
        raise
    
    return model, processor, MODEL_NAME_OR_PATH

model, processor, MODEL_NAME_OR_PATH = load_magma_models()

def generate(image, task_description, template=None, num_marks=10, speed=8, steps=8):
    """TODO: Add docstring."""  
    if template is None:
        template = (
            "<image>\nThe image is split into 256x256 grids and is labeled with numeric marks {}.\n"
            "The robot is doing: {}. To finish the task, how to move the numerical marks in the image "
            "with speed {} for the next {} steps?\n"
        )
    
    mark_ids = [i + 1 for i in range(num_marks)]
    conv_user = template.format(mark_ids, task_description, speed, steps)
    
    if hasattr(model.config, 'mm_use_image_start_end') and model.config.mm_use_image_start_end:
        conv_user = conv_user.replace("<image>", "<image_start><image><image_end>")
    
    convs = [
        {"role": "system", "content": "You are an agent that can see, talk, and act."},
        {"role": "user", "content": conv_user},
    ]
    
    prompt = processor.tokenizer.apply_chat_template(
        convs,
        tokenize=False,
        add_generation_prompt=True
    )
    
    try:
        inputs = processor(images=image, texts=prompt, return_tensors="pt")
        inputs['pixel_values'] = inputs['pixel_values'].unsqueeze(0)
        inputs['image_sizes'] = inputs['image_sizes'].unsqueeze(0)
        inputs = inputs.to(model.device)
        
        with torch.inference_mode():
            output_ids = model.generate(
                **inputs,
                temperature=0.3,
                do_sample=True,
                num_beams=1,
                max_new_tokens=1024,
                use_cache=True,
            )
        response = processor.batch_decode(output_ids, skip_special_tokens=True)[0]
        return response
    except Exception as e:
        logger.error(f"Error in generate: {e}")
        return f"Error: {e}"

def main():
    """TODO: Add docstring."""  
    node = Node()
    frames = {} 
    
    for event in node:
        event_type = event["type"]
        
        if event_type == "INPUT":
            event_id = event["id"]
            
            if "image" in event_id:
                storage = event["value"]
                metadata = event["metadata"]
                encoding = metadata["encoding"]
                width = metadata["width"]
                height = metadata["height"]
                
                try:
                    if encoding == "bgr8":
                        frame = storage.to_numpy().astype(np.uint8).reshape((height, width, 3))
                        frame = frame[:, :, ::-1]  # Convert BGR to RGB
                    elif encoding == "rgb8":
                        frame = storage.to_numpy().astype(np.uint8).reshape((height, width, 3))
                    elif encoding in ["jpeg", "jpg", "jpe", "bmp", "webp", "png"]:
                        storage = storage.to_numpy()
                        frame = cv2.imdecode(storage, cv2.IMREAD_COLOR)
                        if frame is None:
                            raise ValueError(f"Failed to decode image with encoding {encoding}")
                        frame = frame[:, :, ::-1]  # Convert BGR to RGB
                    else:
                        raise ValueError(f"Unsupported image encoding: {encoding}")
                    
                    image = Image.fromarray(frame)
                    frames[event_id] = image
                    
                    # Cleanup old frames
                    if len(frames) > 10:
                        frames.popitem(last=False)
                except Exception as e:
                    logger.error(f"Error processing image {event_id}: {e}")
            
            # Handle text inputs
            elif "text" in event_id:
                if len(event["value"]) > 0:
                    task_description = event["value"][0].as_py()
                    image_id = event["metadata"].get("image_id", None)
                    
                    if image_id is None or image_id not in frames:
                        logger.error(f"Image ID {image_id} not found in frames")
                        continue
                    
                    image = frames[image_id]
                    response = generate(image, task_description)
                    node.send_output(
                        "text",
                        pa.array([response]),
                        {"image_id": image_id}
                    )
                else:
                    continue
        
        elif event_type == "ERROR":
            logger.error(f"Event Error: {event['error']}")

if __name__ == "__main__":
    main()