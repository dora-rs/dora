import os
import json
from dora import Node
import numpy as np
import pyarrow as pa
from PIL import Image
from pathlib import Path

DEFAULT_QUESTION = os.getenv(
    "DEFAULT_QUESTION",
    "Describe this image",
)
ENTRY_NAME = "dora_demo"
LLAMA_FACTORY_ROOT_PATH = Path(os.getenv("LLAMA_FACTORY_ROOT_PATH")) / "data"


# If JSON already exists, append incremental suffix to avoid overwriting
if (LLAMA_FACTORY_ROOT_PATH / ENTRY_NAME).exists():
    i = 1
    while (LLAMA_FACTORY_ROOT_PATH / f"{ENTRY_NAME}_{i}.json").exists():
        i += 1
    ENTRY_NAME = f"{ENTRY_NAME}_{i}"


DEFAULT_RECORD_IMAGE_ROOT_PATH = LLAMA_FACTORY_ROOT_PATH / ENTRY_NAME
DEFAULT_RECORD_JSON_PATH = LLAMA_FACTORY_ROOT_PATH / (ENTRY_NAME + ".json")


def write_dict_to_json(file_path, key: str, new_data):
    """
    Writes a dictionary to a JSON file. If the file already contains a list of entries,
    the new data will be appended to that list. Otherwise, it will create a new list.

    Parameters:
    - file_path: str, the path to the JSON file.
    - new_data: dict, the dictionary to add to the JSON file.
    """
    try:
        # Open the JSON file and load its content
        with open(file_path, "r+", encoding="utf-8") as file:
            try:
                data = json.load(file)
            except json.JSONDecodeError:
                data = {}

            data[key] = new_data
            # Write the updated data back to the file
            file.seek(0)
            json.dump(data, file, indent=4, ensure_ascii=False)
            file.truncate()

    except FileNotFoundError:
        # If the file doesn't exist, create it and write the new data as a list
        with open(file_path, "w", encoding="utf-8") as file:
            json.dump({key: new_data}, file, indent=4, ensure_ascii=False)


write_dict_to_json(
    LLAMA_FACTORY_ROOT_PATH / "dataset_info.json",
    ENTRY_NAME,
    {
        "file_name": ENTRY_NAME + ".json",
        "formatting": "sharegpt",
        "columns": {"messages": "messages", "images": "images"},
        "tags": {
            "role_tag": "role",
            "content_tag": "content",
            "user_tag": "user",
            "assistant_tag": "assistant",
        },
    },
)


def save_image_and_add_to_json(
    image_array, root_path, llama_root_path, jsonl_file, messages
):
    """
    Saves an image from a NumPy array and adds a new JSON object as a line to a JSONL file.
    The function generates a sequential numeric image filename starting from 0 and
    follows the provided template structure.

    Parameters:
    - image_array: numpy.ndarray, the image data as a NumPy array.
    - root_path: str, the root directory where the image will be saved.
    - jsonl_file: str, the path to the JSONL file.
    - messages: list of dicts, each containing 'content' and 'role'.

    The image is saved as a PNG file, and the JSONL entry includes the 'messages' and 'images' keys.
    """

    # Create the root directory if it doesn't exist
    os.makedirs(llama_root_path / root_path, exist_ok=True)

    # Get the current image ID by counting existing files
    image_id = len(
        [
            name
            for name in os.listdir(llama_root_path / root_path)
            if os.path.isfile(os.path.join(llama_root_path / root_path, name))
        ]
    )

    # Define the image filename
    image_filename = f"{image_id}.png"
    image_path = os.path.join(root_path, image_filename)

    # Save the image
    image = Image.fromarray(image_array)
    image.save(llama_root_path / image_path)

    # Create the JSON entry with 'messages' and 'images'
    new_entry = {"messages": messages, "images": [image_path]}

    # Add the entry to the JSONL file with UTF-8 encoding
    with open(jsonl_file, "a", encoding="utf-8") as f:
        json_line = json.dumps(new_entry)
        f.write(json_line + "\n")


def main():
    pa.array([])  # initialize pyarrow array
    node = Node()

    question = DEFAULT_QUESTION
    frame = None

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

            elif event_id == "text":
                text = event["value"][0].as_py()
                if text != "":
                    question = text
            elif event_id == "ground_truth":
                if frame is None:
                    continue
                ground_truth = event["value"][0].as_py()

                messages = [
                    {"content": "<image>" + question, "role": "user"},
                    {
                        "content": ground_truth,
                        "role": "assistant",
                    },
                ]

                save_image_and_add_to_json(
                    image_array=frame,
                    root_path=ENTRY_NAME,
                    llama_root_path=LLAMA_FACTORY_ROOT_PATH,
                    jsonl_file=DEFAULT_RECORD_JSON_PATH,
                    messages=messages,
                )
                node.send_output(
                    "text",
                    pa.array([ground_truth]),
                    metadata,
                )

        elif event_type == "ERROR":
            raise RuntimeError(event["error"])
