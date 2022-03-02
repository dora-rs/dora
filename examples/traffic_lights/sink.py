import os
import time

import cv2
import numpy as np

from dora import register

SRC_LABELS = os.environ["OP_LABELS"]


@register(SRC_LABELS)
def listener(state, change):
    """
    Publication of change
    """
    value = bytes(change.value.get_content())

    array = np.frombuffer(value, dtype=np.uint8)

    array = array.reshape((587, 1043, 3))
    array = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
    # cv2.imshow("test", array)

    # cv2.waitKey(0)
    print("recieved")


if __name__ == "__main__":
    time.sleep(60)
