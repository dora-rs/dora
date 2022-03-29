import cv2

IMAGE_PATH = "examples/pylot/panneau-feu-usa2.jpg"


src = cv2.imread(IMAGE_PATH).tobytes()


def produce(image=None):
    """
    read and produce a temperature every second
    """

    return {"image": src}
