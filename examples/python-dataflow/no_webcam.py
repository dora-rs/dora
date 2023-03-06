#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import urllib.request

import cv2
import numpy as np
from dora import Node

req = urllib.request.urlopen("https://ultralytics.com/images/zidane.jpg")

arr = np.asarray(bytearray(req.read()), dtype=np.uint8)
node = Node()

start = time.time()

while time.time() - start < 20:
    # Wait next dora_input
    event = node.next()
    match event["type"]:
        case "INPUT":
            print("received input", event["id"])
            node.send_output("image", arr.tobytes())
        case "STOP":
            print("received stop")
        case other:
            print("received unexpected event:", other)
