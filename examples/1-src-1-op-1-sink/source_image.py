import os
import time

import cv2
import zenoh

TRAFFIC_LIGHT_DET_MIN_SCORE_THRESHOLD = 0.001
WIDTH = 1043
HEIGHT = 587
GPU_DEVICE = 0
LABELS = os.environ["LABELS"]
z = zenoh.open()


IMAGE_PATH = "../data/carla_example.png"
import numpy as np


def sensor_loop():
    """
    read and produce a temperature every second
    """

    src = cv2.imread(IMAGE_PATH)
    z.put(LABELS, src.tobytes())

    time.sleep(1)


if __name__ == "__main__":
    while True:
        sensor_loop()
