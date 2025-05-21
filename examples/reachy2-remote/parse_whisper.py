"""TODO: Add docstring."""

import os
import time
import re

import numpy as np
import pyarrow as pa
from dora import Node


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

    elif (
        ("pick" in text and "place" in text)
        or ("make a hot dog" in text)
        or ("make a vegetarian hot dog" in text)
        or ("cook" in text)
    ):
        if "make a hot dog" in text:
            text = "pick the sausage, place it on the black grill, wait, flip the sausage on the grill, wait, pick the sausage on the grill, place it on the hot dog bun, speak it's ready!"
        if "make a vegetarian hot dog" in text:
            text = "pick the cucumber, place it on the black grill, wait, flip the cucumber on the grill, wait, pick the cucumber on the grill, place it on the hot dog bun, speak it's ready!"
        elif "cook" in text:
            # Match 'grill' followed by two words
            match = re.search(r"\bcook\b\s+(\w+)\s+(\w+)", text)

            if match:
                word1 = match.group(1)
                word2 = match.group(2)
                grilled_item = word1 + " " + word2
                text = f"pick {grilled_item}, place it on the black grill, wait, flip {grilled_item}, wait, pick {grilled_item} again, place it on the white plate, speak {grilled_item} is ready!"

        if "," or "." in text:
            prompts = re.split(r"[,.]", text)
            queue = prompts
            first_prompt = queue[0]
            queue = queue[1:]
            handle_event(first_prompt)

    elif "pick " in text:
        text = text.replace("can you", "")
        text = text.replace("please", "")
        text = text.replace("reachy", "")

        node.send_output("speech", pa.array(["I'm going to " + text]))

        text = f"Given the prompt: {text}. Output the bounding boxes for the object to be picked"

        node.send_output(
            "text",
            pa.array([text]),
            {"image_id": "image_depth", "action": "pick"},
        )
    elif "place " in text:
        text = text.replace("can you", "")
        text = text.replace("please", "")
        text = text.replace("reachy", "")

        node.send_output("speech", pa.array(["I'm going to " + text]))

        text = f"Given the prompt: {text}. Output the bounding boxes for the place to place the object"
        node.send_output(
            "text",
            pa.array([text]),
            {"image_id": "image_depth", "action": "release"},
        )
    elif " wait" in text:
        node.send_output("speech", pa.array(["I'm going to wait for 5 seconds."]))

        time.sleep(5)
        if len(queue) > 0:
            first_prompt = queue[0]
            queue = queue[1:]
            handle_event(first_prompt)
    elif " speak" in text:
        node.send_output("speech", pa.array([text.replace("speak ", "")]))

        if len(queue) > 0:
            first_prompt = queue[0]
            queue = queue[1:]
            handle_event(first_prompt)
    elif " flip" in text:
        text = text.replace("can you", "")
        text = text.replace("please", "")
        text = text.replace("reachy", "")

        node.send_output("speech", pa.array(["I'm going to " + text]))

        text = f"Given the prompt: {text}. Output the bounding boxes for the object to flip"
        node.send_output(
            "text",
            pa.array([text]),
            {"image_id": "image_depth", "action": "flip"},
        )
    # elif "flip " in text:
    #    node.send_output("flip", pa.array([True]))
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
