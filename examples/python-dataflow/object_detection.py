#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from enum import Enum
from typing import Callable
from dora import Node

import cv2
import numpy as np
import torch

model = torch.hub.load("ultralytics/yolov5", "yolov5n")

node = Node()

for event in node:
    match event["type"]:
        case "input":
            match event["id"]:
                case "image":
                    print("received image input")
                    frame = np.frombuffer(event["data"], dtype="uint8")
                    frame = cv2.imdecode(frame, -1)
                    frame = frame[:, :, ::-1]  # OpenCV image (BGR to RGB)
                    results = model(frame)  # includes NMS
                    arrays = np.array(results.xyxy[0].cpu()).tobytes()

                    node.send_output("bbox", arrays, event["metadata"])
                case other:
                    print("ignoring unexpected input:", other)
        case "stop":
            print("received stop")
            break
        case other:
            print("received unexpected event:", other)
            break
