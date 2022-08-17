#!/usr/bin/env python
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
    # Wait next input
    node.next()
    node.send_output("image", arr.tobytes())

time.sleep(1)
