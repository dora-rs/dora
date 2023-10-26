#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from typing import Callable
from dora import Node

import cv2
import numpy as np
import torch

model = torch.hub.load("ultralytics/yolov5", "yolov5n")

node = Node()

for event in node:
    match event["type"]:
        case "INPUT":
            match event["id"]:
                case "image":
                    print("[object detection] received image input")
                    frame = event["value"].to_numpy()
                    frame = cv2.imdecode(frame, -1)
                    frame = frame[:, :, ::-1]  # OpenCV image (BGR to RGB)
                    results = model(frame)  # includes NMS
                    arrays = np.array(results.xyxy[0].cpu()).tobytes()

                    node.send_output("bbox", arrays, event["metadata"])
                case other:
                    print("[object detection] ignoring unexpected input:", other)
        case "STOP":
            print("[object detection] received stop")
        case "ERROR":
            print("[object detection] error: ", event["error"])
        case other:
            print("[object detection] received unexpected event:", other)
