#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time

import cv2

from dora import Node

node = Node()

video_capture = cv2.VideoCapture(0)

start = time.time()

# Run for 20 seconds
while time.time() - start < 10:
    # Wait next dora_input
    event = node.next()
    match event["type"]:
        case "INPUT":
            ret, frame = video_capture.read()
            if ret:
                node.send_output(
                    "image",
                    cv2.imencode(".jpg", frame)[1].tobytes(),
                    event["metadata"],
                )
        case "STOP":
            print("received stop")
            break
        case other:
            print("received unexpected event:", other)
            break

video_capture.release()
