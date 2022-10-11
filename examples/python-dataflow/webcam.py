#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time

import cv2
from dora import Node

node = Node()

video_capture = cv2.VideoCapture(0)

start = time.time()

# Run for 20 seconds
while time.time() - start < 20:
    # Wait next dora_input
    node.next()
    ret, frame = video_capture.read()
    if ret:
        node.send_output("image", cv2.imencode(".jpg", frame)[1].tobytes())

video_capture.release()
