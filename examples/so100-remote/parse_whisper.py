"""TODO: Add docstring."""

import json
import os
import time

import numpy as np
import pyarrow as pa
from dora import Node

node = Node()


last_prompt = ""
for event in node:
    if event["type"] == "INPUT":
        if event["id"] == "text":
            text = event["value"][0].as_py().lower()

            if "grab " in text:
                text = f"Given the prompt: {text}. Output the bounding boxes for the given object"
                node.send_output(
                    "text", pa.array([text]), {"image_id": "image", "action": "grab"}
                )

            elif "put " in text:
                text = f"Given the prompt: {text}. Output the bounding boxes for the place to put the object"
                node.send_output(
                    "text",
                    pa.array([text]),
                    {"image_id": "image", "action": "release"},
                )

            elif "make a hot dog" in text:
                text = f"Given the prompt: grab the sausage. Output the bounding boxes for the given object"
                node.send_output(
                    "text", pa.array([text]), {"image_id": "image", "action": "grab"}
                )
                time.sleep(4.0)

                text = f"Given the prompt: put it in the black cooking grill. Output the bounding boxes for the given object"
                node.send_output(
                    "text", pa.array([text]), {"image_id": "image", "action": "release"}
                )
                time.sleep(3.0)

                text = f"Given the prompt: grab the sausage. Output the bounding boxes for the given object"
                node.send_output(
                    "text", pa.array([text]), {"image_id": "image", "action": "grab"}
                )
                time.sleep(1.6)
                
                text = f"Given the prompt: put it in the slice of bread. Output the bounding boxes for the given object"
                node.send_output(
                    "text", pa.array([text]), {"image_id": "image", "action": "release"}
                )
