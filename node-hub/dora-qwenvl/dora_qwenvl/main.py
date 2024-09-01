import os
from dora import Node
from transformers import Qwen2VLForConditionalGeneration, AutoProcessor
from qwen_vl_utils import process_vision_info
import numpy as np
import pyarrow as pa
from PIL import Image

DEFAULT_PATH = "Qwen/Qwen2-VL-2B-Instruct"
CUSTOM_MODEL_PATH = os.getenv("CUSTOM_MODEL_PATH", DEFAULT_PATH)
DEFAULT_QUESTION = os.getenv(
    "DEFAULT_QUESTION",
    "Describe this image",
)

# default: Load the model on the available device(s)
model = Qwen2VLForConditionalGeneration.from_pretrained(
    PATH,
    torch_dtype="auto",
    device_map="auto",
    attn_implementation="flash_attention_2",
)

# default processer
processor = AutoProcessor.from_pretrained(DEFAULT_PATH)


def generate(image: np.array, question):
    """
    Generate the response to the question given the image using Qwen2 model.
    """
    image = Image.fromarray(image)

    messages = [
        {
            "role": "user",
            "content": [
                {
                    "type": "image",
                    "image": image,
                },
                {"type": "text", "text": question},
            ],
        }
    ]

    # Preparation for inference
    text = processor.apply_chat_template(
        messages, tokenize=False, add_generation_prompt=True
    )
    image_inputs, video_inputs = process_vision_info(messages)
    inputs = processor(
        text=[text],
        images=image_inputs,
        videos=video_inputs,
        padding=True,
        return_tensors="pt",
    )
    inputs = inputs.to("cuda")

    # Inference: Generation of the output
    generated_ids = model.generate(**inputs, max_new_tokens=128)
    generated_ids_trimmed = [
        out_ids[len(in_ids) :]
        for in_ids, out_ids in zip(inputs.input_ids, generated_ids)
    ]
    output_text = processor.batch_decode(
        generated_ids_trimmed,
        skip_special_tokens=True,
        clean_up_tokenization_spaces=False,
    )
    return output_text[0]


def main():
    node = Node()

    question = DEFAULT_QUESTION
    frame = None
    pa.array([])  # initialize pyarrow array

    for event in node:
        event_type = event["type"]

        if event_type == "INPUT":
            event_id = event["id"]

            if event_id == "image":
                storage = event["value"]
                metadata = event["metadata"]
                encoding = metadata["encoding"]
                width = metadata["width"]
                height = metadata["height"]

                if encoding == "bgr8":
                    channels = 3
                    storage_type = np.uint8
                elif encoding == "rgb8":
                    channels = 3
                    storage_type = np.uint8
                else:
                    raise RuntimeError(f"Unsupported image encoding: {encoding}")

                frame = (
                    storage.to_numpy()
                    .astype(storage_type)
                    .reshape((height, width, channels))
                )
                if encoding == "bgr8":
                    frame = frame[:, :, ::-1]  # OpenCV image (BGR to RGB)
                elif encoding == "rgb8":
                    pass
                else:
                    raise RuntimeError(f"Unsupported image encoding: {encoding}")

            elif event_id == "tick":
                if frame is None:
                    continue
                response = generate(frame, question)
                node.send_output(
                    "text",
                    pa.array([response]),
                    metadata,
                )

            elif event_id == "text":
                text = event["value"][0].as_py()
                if text != "":
                    question = text
                if frame is None:
                    continue
                # set the max number of tiles in `max_num`
                response = generate(frame, question)
                node.send_output(
                    "text",
                    pa.array([response]),
                    metadata,
                )

        elif event_type == "ERROR":
            raise RuntimeError(event["error"])


if __name__ == "__main__":
    main()
