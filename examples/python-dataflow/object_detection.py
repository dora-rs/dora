#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
from ultralytics import YOLO

from dora import Node
import pyarrow as pa

model = YOLO("yolov8n.pt")

try:
    node = Node()
except RuntimeError as e:
    print("Dataflow initialization failed: ", e)
    exit(0)


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
            # Process results
            boxes = np.array(results[0].boxes.xyxy.cpu())
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
