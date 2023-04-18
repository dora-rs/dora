#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from typing import Callable

import cv2
import pyarrow as pa

from dora import DoraStatus

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480


class Operator:
    """
    Sending image from webcam to the dataflow
    """

    def __init__(self):
        self.video_capture = cv2.VideoCapture(0)
        self.start_time = time.time()
        self.video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self.video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

    def on_event(
        self,
        dora_event: str,
        send_output: Callable[[str, bytes], None],
    ) -> DoraStatus:
        match dora_event["type"]:
            case "INPUT":
                ret, frame = self.video_capture.read()
                frame = cv2.resize(frame, (CAMERA_WIDTH, CAMERA_HEIGHT))
                if ret:
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
