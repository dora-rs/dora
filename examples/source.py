import time
import cv2

IMAGE_PATH = "examples/panneau-feu-usa2.jpg"


src = cv2.imread(IMAGE_PATH)


def produce(b, a=None):
    """
    read and produce a temperature every second
    """

    return src.tobytes()
