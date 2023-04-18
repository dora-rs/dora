#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import urllib.request

import cv2
import numpy as np
import pyarrow as pa

from dora import Node

print("Hello from no_webcam.py")


CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

# Preprocessing the image
req = urllib.request.urlopen(
    "https://img0.baidu.com/it/u=2940037857,1417768899&fm=253&fmt=auto&app=138&f=PNG?w=724&h=500"
)  # This image works in china better
arr = np.asarray(bytearray(req.read()), dtype=np.uint8)
image = cv2.imdecode(arr, -1)[:, :, :3]
image = cv2.resize(image, (CAMERA_WIDTH, CAMERA_HEIGHT))

# Numpy -> Arrow
image = pa.array(image.flatten().view(np.uint8))
node = Node()

start = time.time()

while time.time() - start < 20:
    # Wait next dora_input
    event = node.next()
    match event["type"]:
        case "INPUT":
            print("received input", event["id"])
            node.send_output("image", image)
        case "STOP":
            print("received stop")
        case other:
            print("received unexpected event:", other)
