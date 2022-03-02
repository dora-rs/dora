import os
import time

import cv2
import zenoh

TRAFFIC_LIGHT_DET_MIN_SCORE_THRESHOLD = 0.001
WIDTH = 1043
HEIGHT = 587
GPU_DEVICE = 0
LABELS = os.environ["SRC_LABELS"]
z = zenoh.open()


IMAGE_PATH = "../data/panneau-feu-usa2.jpg"


def sensor_loop():
    """
    read and produce a temperature every second
    """

    src = cv2.imread(IMAGE_PATH)

    dst = cv2.cvtColor(src, cv2.COLOR_BGR2RGB)
    info = z.info()["info_pid"]
    z.put(LABELS + "/" + info, dst.tobytes())

    time.sleep(1)


if __name__ == "__main__":
    while True:
        sensor_loop()
