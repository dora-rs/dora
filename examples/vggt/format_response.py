"""TODO: Add docstring."""

import base64
from dora import Node
import pyarrow as pa
import numpy as np
import cv2
node = Node()


cache = {}

def derive_image_as_base64(event):
    storage = event["value"]
    metadata = event["metadata"]
    encoding = metadata["encoding"]
    width = metadata["width"]
    height = metadata["height"]

    if (
        encoding == "mono16" or encoding == "z16"
    ):
        channels = 1
        storage_type = np.uint16
    else:
        channels = 3
        storage_type = np.uint8

    if encoding == "bgr8":
        frame = (
            storage.to_numpy()
            .astype(storage_type)
            .reshape((height, width, channels))
        )
        frame = cv2.imencode('.jpg', frame)[1].tobytes()
        frame = base64.b64encode(frame).decode('utf-8')
        image = f"data:image/jpeg;base64,{frame}"
    elif encoding == "rgb8":
        frame = (
            storage.to_numpy()
            .astype(storage_type)
            .reshape((height, width, channels))
        )
        frame = frame[:, :, ::-1]  # OpenCV image (RGB to BGR)
        frame = cv2.imencode('.jpg', frame)[1].tobytes()
        frame = base64.b64encode(frame).decode('utf-8')
        image = f"data:image/jpeg;base64,{frame}"
    elif encoding in ["jpeg", "jpg", "avif", "webp", "png", "bmp"]:
        storage = storage.to_numpy()
        image = base64.b64encode(storage).decode('utf-8')
        image = f"data:image/{encoding};base64,{image}"
    elif encoding in ["mono16", "z16"]:
        frame = (
            storage.to_numpy()
            .astype(storage_type)
            .reshape((height, width, 1))
        )
        storage = storage.to_numpy()
        image = base64.b64encode(storage).decode('utf-8')
        image = f"data:image/jpeg;base64,{frame}"

    else:
        raise RuntimeError(f"Unsupported image encoding: {encoding}")

    return image

for event in node:
    if event["type"] == "INPUT":
        event_id = event["id"]
        print(f"Received event: {event_id}")
        match event_id:
            case s if "image" in s:
                
                message_id = event["metadata"].get("id", 0)
                if cache.get(message_id) is None:
                    cache[message_id] = {}
                cache[message_id]["image"] = event
            case s if "depth" in s:
                message_id = event["metadata"].get("id", 0)
                if cache.get(message_id) is None:
                    cache[message_id] = {}
                cache[message_id]["depth"] = event
        if len(cache[message_id]) == 2:
            image = cache[message_id]["image"]
            image_str = derive_image_as_base64(image)
            depth = cache[message_id]["depth"]
            depth_str = derive_image_as_base64(depth)

            node.send_output(
                "text",
                pa.array(np.array(image_str + "\n" + depth_str).ravel()),
                metadata={
                    "message_id": message_id,
                },
            )
