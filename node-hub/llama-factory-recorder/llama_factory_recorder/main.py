import os
import json
from dora import Node
import numpy as np
import pyarrow as pa
from PIL import Image
from pathlib import Path
import cv2

DEFAULT_QUESTION = os.getenv(
    "DEFAULT_QUESTION",
    "Describe this image",
)


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


def save_image_and_add_to_json(
    frame_dict: dict, root_path, llama_root_path, jsonl_file, messages
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
    image_paths = []
    for event_id, data in frame_dict.items():
        # Define the image filename
        image_filename = f"{event_id}-{image_id}.png"
        image_path = os.path.join(root_path, image_filename)

        # Save the image
        image = Image.fromarray(data)
        image.save(llama_root_path / image_path)
        image_paths.append(image_path)

    # Create the JSON entry with 'messages' and 'images'
    new_entry = {"messages": messages, "images": image_paths}

    # Add the entry to the JSONL file with UTF-8 encoding
    with open(jsonl_file, "a", encoding="utf-8") as f:
        json_line = json.dumps(new_entry)
        f.write(json_line + "\n")


def main():
    pa.array([])  # initialize pyarrow array
    node = Node()

    assert os.getenv(
        "LLAMA_FACTORY_ROOT_PATH"
    ), "LLAMA_FACTORY_ROOT_PATH is not set, Either git clone the repo or set the environment variable"
    llama_factory_root_path = Path(os.getenv("LLAMA_FACTORY_ROOT_PATH")) / "data"

    entry_name = os.getenv("ENTRY_NAME", "dora_demo")
    # If JSON already exists, append incremental suffix to avoid overwriting
    if (llama_factory_root_path / entry_name).exists():
        i = 1
        while (llama_factory_root_path / f"{entry_name}_{i}.json").exists():
            i += 1
        entry_name = f"{entry_name}_{i}"

    default_record_json_path = llama_factory_root_path / (entry_name + ".json")

    write_dict_to_json(
        llama_factory_root_path / "dataset_info.json",
        entry_name,
        {
            "file_name": entry_name + ".json",
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

    question = DEFAULT_QUESTION
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

                if encoding == "bgr8":
                    channels = 3
                    storage_type = np.uint8
                elif encoding == "rgb8":
                    channels = 3
                    storage_type = np.uint8
                elif encoding == "jpeg":
                    channels = 3
                    storage_type = np.uint8
                else:
                    raise RuntimeError(f"Unsupported image encoding: {encoding}")

                if encoding == "bgr8":
                    frame = (
                        storage.to_numpy()
                        .astype(storage_type)
                        .reshape((height, width, channels))
                    )
                    frame = frame[:, :, ::-1]  # OpenCV image (BGR to RGB)
                elif encoding == "rgb8":
                    frame = (
                        storage.to_numpy()
                        .astype(storage_type)
                        .reshape((height, width, channels))
                    )
                elif encoding in ["jpeg", "jpg", "jpe", "bmp", "webp", "png"]:
                    channels = 3
                    storage_type = np.uint8
                    storage = storage.to_numpy()
                    frame = cv2.imdecode(storage, cv2.IMREAD_COLOR)
                    frame = frame[:, :, ::-1]  # OpenCV image (BGR to RGB)
                else:
                    raise RuntimeError(f"Unsupported image encoding: {encoding}")

                frames[event_id] = frame

            elif event_id == "text":
                text = event["value"][0].as_py()
                if text != "":
                    question = text
            elif event_id == "ground_truth":
                if len(frames.keys()) == 0:
                    continue
                ground_truth = event["value"][0].as_py()

                messages = [
                    {
                        "content": "<image>" * len(frames.keys()) + question,
                        "role": "user",
                    },
                    {
                        "content": ground_truth,
                        "role": "assistant",
                    },
                ]

                save_image_and_add_to_json(
                    frame_dict=frames,
                    root_path=entry_name,
                    llama_root_path=llama_factory_root_path,
                    jsonl_file=default_record_json_path,
                    messages=messages,
                )
                node.send_output(
                    "text",
                    pa.array([ground_truth]),
                    metadata,
                )

        elif event_type == "ERROR":
            raise RuntimeError(event["error"])
