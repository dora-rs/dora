#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import numpy as np
import cv2

from dora import Node
import pyarrow as pa

node = Node()

IMAGE_INDEX = int(os.getenv("IMAGE_INDEX", 0))
IMAGE_WIDTH = int(os.getenv("IMAGE_WIDTH", 960))
IMAGE_HEIGHT = int(os.getenv("IMAGE_HEIGHT", 540))
video_capture = cv2.VideoCapture(IMAGE_INDEX)
video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, IMAGE_WIDTH)
video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT)
font = cv2.FONT_HERSHEY_SIMPLEX

start = time.time()

# Run for 20 seconds
while time.time() - start < 1000:
    # Wait next dora_input
    event = node.next()
    if event is None:
        break

    event_type = event["type"]
    if event_type == "INPUT":
        ret, frame = video_capture.read()
        if not ret:
            frame = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH, 3), dtype=np.uint8)
            cv2.putText(
                frame,
                "No Webcam was found at index %d" % (IMAGE_INDEX),
                (int(30), int(30)),
                font,
                0.75,
                (255, 255, 255),
                2,
                1,
            )
        if len(frame) != IMAGE_HEIGHT * IMAGE_WIDTH * 3:
            print("frame size is not correct")
            frame = cv2.resize(frame, (IMAGE_WIDTH, IMAGE_HEIGHT))

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        node.send_output(
            "image",
            pa.array(frame.ravel()),
            event["metadata"],
        )
        node.send_output("text", pa.array([f"send image at: {time.time()}"]))
