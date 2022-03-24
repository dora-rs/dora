import numpy as np

import cv2

import zenoh
import os

z = zenoh.open()
LABELS = os.environ["LABELS"]
IMAGE_PATH = "examples/panneau-feu-usa2.jpg"


def sensor_loop():
    """
    read and produce a temperature every second
    """

    src = cv2.imread(IMAGE_PATH)
    z.put(LABELS, src.tobytes())


sensor_loop()
