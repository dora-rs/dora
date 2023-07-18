#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from typing import Callable

import os
import cv2
import numpy as np
import pyarrow as pa

from dora import DoraStatus

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_INDEX = os.getenv("CAMERA_INDEX", 0)

font = cv2.FONT_HERSHEY_SIMPLEX


class Operator:
    """
    Sending image from webcam to the dataflow
    """

    def __init__(self):
        self.video_capture = cv2.VideoCapture(CAMERA_INDEX)
        self.start_time = time.time()
        self.video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self.video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

    def on_event(
        self,
        dora_event: str,
        send_output: Callable[[str, bytes | pa.UInt8Array, dict], None],
    ) -> DoraStatus:
        match dora_event["type"]:
            case "INPUT":
                ret, frame = self.video_capture.read()
                if ret:
                    frame = cv2.resize(frame, (CAMERA_WIDTH, CAMERA_HEIGHT))

                ## Push an error image in case the camera is not available.
                else:
                    frame = np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.uint8)
                    cv2.putText(
                        frame,
                        "No Webcam was found at index %d" % (CAMERA_INDEX),
                        (int(30), int(30)),
                        font,
                        0.75,
                        (255, 255, 255),
                        2,
                        1,
                    )

                send_output(
                    "image",
                    pa.array(frame.ravel()),
                    dora_event["metadata"],
                )
            case "STOP":
                print("received stop")
            case other:
                print("received unexpected event:", other)

        if time.time() - self.start_time < 20:
            return DoraStatus.CONTINUE
        else:
            return DoraStatus.STOP

    def __del__(self):
        self.video_capture.release()
