"""TODO: Add docstring."""

import json
import os

import numpy as np
import pyarrow as pa
from dora import Node

node = Node()

IMAGE_RESIZE_RATIO = float(os.getenv("IMAGE_RESIZE_RATIO", "1.0"))


for event in node:
    if event["type"] == "INPUT":
        text = event["value"][0].as_py()
        width = event["metadata"]["width"]
        height = event["metadata"]["height"]
        values = event["value"].to_numpy().reshape((-1, 2))
        values = values * int(1 / IMAGE_RESIZE_RATIO)

        # Do point 0 first
        if len(values) == 0:
            print("No points detected")
            continue
        elif len(values) > 1:
            print("Multiple points detected, taking the first one")
        point = values[-1]

        rz = int((width / 2) - point[0]) / (width / 2)
        x_distance = min(height, height - point[1])

        if abs(rz) > 0.75:
            rz = np.deg2rad(75) * np.sign(rz)
        if abs(rz) > 0.5:
            rz = np.deg2rad(50) * np.sign(rz)
        elif abs(rz) > 0.3:
            rz = np.deg2rad(45) * np.sign(rz)
        elif abs(rz) > 0.1:
            rz = np.deg2rad(30) * np.sign(rz)
        else:
            x = 0

        if x_distance > (height * 0.7):
            x = 1.0
        elif x_distance > (height * 0.5):
            x = 0.7
        elif x_distance > (height * 0.2):
            x = 0.5
        else:
            x = 0
        # Action
        action = pa.array([x, 0, 0, 0, 0, rz])
        node.send_output("action", action)
