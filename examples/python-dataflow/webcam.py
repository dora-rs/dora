#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import numpy as np
import cv2

from dora import Node

node = Node()

CAMERA_INDEX = int(os.getenv("CAMERA_INDEX", 0))
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
video_capture = cv2.VideoCapture(CAMERA_INDEX)
font = cv2.FONT_HERSHEY_SIMPLEX

start = time.time()

# Run for 20 seconds
while time.time() - start < 10:
    # Wait next dora_input
    event = node.next()
    event_type = event["type"]
    if event_type == "INPUT":
        ret, frame = video_capture.read()
        if not ret:
            frame = np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.uint8)
            cv2.putText(
                frame,
                "No Webcam was found at index %d" % (CAMERA_INDEX),
                (int(30), int(30)),
                font,
                0.75,
                (255, 255, 255),
                2,
                1,
            )
        node.send_output(
            "image",
            cv2.imencode(".jpg", frame)[1].tobytes(),
            event["metadata"],
        )
    elif event_type == "INPUT":
        print("received stop")
        break
    else:
        print("received unexpected event:", event_type)
        break

video_capture.release()
