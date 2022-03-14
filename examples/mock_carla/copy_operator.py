import os
import time

import cv2
import numpy as np

from dora import register


@register(os.environ["NAME"])
def detect_lane(state, change):

    value = bytes(change)
    image = np.frombuffer(value, dtype=np.dtype("uint8"))
    image = np.reshape(image, (720, 1280, 3))
    image = np.reshape(image, (720, 1280, 3))[:, :, :3]
    image = cv2.resize(image, (512, 256), interpolation=cv2.INTER_LINEAR)
    return image.tobytes()


if __name__ == "__main__":
    while True:
        time.sleep(5)
