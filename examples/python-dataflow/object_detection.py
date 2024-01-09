#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import cv2
import numpy as np
import torch

from dora import Node

# Reload only if on Windows
reload = os.name == "nt"
model = torch.hub.load("ultralytics/yolov5", "yolov5n", force_reload=reload)

node = Node()

for event in node:
    event_type = event["type"]
    if event_type == "INPUT":
        event_id = event["id"]
        if event_id == "image":
            print("[object detection] received image input")
            frame = event["value"].to_numpy()
            frame = cv2.imdecode(frame, -1)
            frame = frame[:, :, ::-1]  # OpenCV image (BGR to RGB)
            results = model(frame)  # includes NMS
            arrays = np.array(results.xyxy[0].cpu()).tobytes()

            node.send_output("bbox", arrays, event["metadata"])
        else:
            print("[object detection] ignoring unexpected input:", event_id)
    elif event_type == "STOP":
        print("[object detection] received stop")
    elif event_type == "ERROR":
        print("[object detection] error: ", event["error"])
    else:
        print("[object detection] received unexpected event:", event_type)
