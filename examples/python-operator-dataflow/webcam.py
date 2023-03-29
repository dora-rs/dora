#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from typing import Callable

import cv2

from dora import DoraStatus


class Operator:
    """
    Sending image from webcam to the dataflow
    """

    def __init__(self):
        self.video_capture = cv2.VideoCapture(0)
        self.start_time = time.time()

    def on_event(
        self,
        dora_event: dict,
        send_output: Callable[[str, bytes], None],
    ) -> DoraStatus:
        match dora_event["type"]:
            case "INPUT":
                ret, frame = self.video_capture.read()
                if ret:
                    send_output(
                        "image",
                        cv2.imencode(".jpg", frame)[1].tobytes(),
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
