"""TODO: Add docstring."""

import json
import os
import time

import numpy as np
import pyarrow as pa
from dora import Node

import re

node = Node()

IMAGE_RESIZE_RATIO = float(os.getenv("IMAGE_RESIZE_RATIO", "1.0"))

queue = []

last_prompt = ""

def handle_event(text: str):
    global queue
    if "stop" in text:
        node.send_output("points", pa.array([], type=pa.float64()))
    elif "follow" in text:
        text = f"Given the prompt: {text}. Output the bounding boxes for the given followed object"
        node.send_output("look_ahead", pa.array([1.0]))
        time.sleep(0.5)  # Sync image
        node.send_output("text", pa.array([text]), {"image_id": "image_left"})
        node.send_output("follow_pose", pa.array([1.0]))
    elif "raise your arms" in text:
        node.send_output("raise_arm_pose", pa.array([1.0]))

    elif ("grab" in text and "drop" in text) or ("make a hot dog" in text):
        if "make a hot dog" in text:
            text = "grab the sausage, drop it on the black grill, grab the salad, drop it on the bread, grab the sausage, drop it on the bread"
        if "," or "." in text:
            prompts = re.split(r"[,.]",text)
            queue = prompts
            first_prompt = queue[0]
            queue = queue[1:]
            handle_event(first_prompt)
            
    elif "grab " in text:
        text = f"Given the prompt: {text}. Output the bounding boxes for the given grabbed object"
        node.send_output(
            "text",
            pa.array([text]),
            {"image_id": "image_depth", "action": "grab"},
        )
    elif "drop " in text:
        text = f"Given the prompt: {text}. Output the bounding boxes for the place to drop the object"
        node.send_output(
            "text",
            pa.array([text]),
            {"image_id": "image_depth", "action": "release"},
        )

    elif "release left" in text:
        node.send_output("action_release_left", pa.array([1.0]))
    elif "release right" in text:
        node.send_output("action_release_right", pa.array([1.0]))
    elif "turn left" in text:
        action = pa.array([0.0, 0, 0, 0, 0, np.deg2rad(30)])
        node.send_output("action", action)
        node.send_output("points", pa.array([]))
    elif "turn right" in text:
        action = pa.array([0.0, 0, 0, 0, 0, -np.deg2rad(30)])
        node.send_output("action", action)
        node.send_output("points", pa.array([]))
    elif "turn around" in text:
        action = pa.array([0.0, 0, 0, 0, 0, -np.deg2rad(180)])
        node.send_output("action", action)
        node.send_output("points", pa.array([]))
    elif "move left" in text:
        action = pa.array([0.0, 0.2, 0, 0, 0, 0])
        node.send_output("action", action)
        node.send_output("points", pa.array([]))
    elif "move right" in text:
        action = pa.array([0.0, -0.2, 0, 0, 0, 0])
        node.send_output("action", action)
        node.send_output("points", pa.array([]))
    elif "move forward" in text:
        action = pa.array([0.2, 0, 0, 0, 0, 0])
        node.send_output("action", action)
        node.send_output("points", pa.array([]))
    elif "move backward" in text:
        action = pa.array([-0.2, 0, 0, 0, 0, 0])
        node.send_output("action", action)
        node.send_output("points", pa.array([]))



for event in node:
    if event["type"] == "INPUT":
        if event["id"] == "text":
            event_text = event["value"][0].as_py().lower()
            handle_event(event_text)
        elif event["id"] == "success":
            if len(queue) > 0:
                time.sleep(0.3)
                first_prompt = queue[0]
                queue = queue[1:]
                handle_event(first_prompt)
