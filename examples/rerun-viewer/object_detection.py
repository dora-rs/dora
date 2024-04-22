#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import cv2
import numpy as np
from ultralytics import YOLO

from dora import Node
import pyarrow as pa

model = YOLO("yolov8n.pt")

node = Node()

IMAGE_WIDTH = int(os.getenv("IMAGE_WIDTH", 960))
IMAGE_HEIGHT = int(os.getenv("IMAGE_HEIGHT", 540))

for event in node:
    event_type = event["type"]
    if event_type == "INPUT":
        event_id = event["id"]
        if event_id == "image":
            print("[object detection] received image input")
            image = event["value"].to_numpy().reshape((IMAGE_HEIGHT, IMAGE_WIDTH, 3))

            frame = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            frame = frame[:, :, ::-1]  # OpenCV image (BGR to RGB)
            results = model(frame)  # includes NMS
            # Process results
            boxes = np.array(results[0].boxes.xywh.cpu())
            conf = np.array(results[0].boxes.conf.cpu())
            label = np.array(results[0].boxes.cls.cpu())
            # concatenate them together
            arrays = np.concatenate((boxes, conf[:, None], label[:, None]), axis=1)

            node.send_output("bbox", pa.array(arrays.ravel()), event["metadata"])
        else:
            print("[object detection] ignoring unexpected input:", event_id)
    elif event_type == "STOP":
        print("[object detection] received stop")
    elif event_type == "ERROR":
        print("[object detection] error: ", event["error"])
    else:
        print("[object detection] received unexpected event:", event_type)
