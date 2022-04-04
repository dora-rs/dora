import threading
import time

import cv2
import numpy as np
import pygame

mutex = threading.Lock()
pygame.init()


display_width = 720
display_height = 1280

gameDisplay = pygame.display.set_mode(
    (display_width, display_height),
    pygame.HWSURFACE | pygame.DOUBLEBUF,
)
font = cv2.FONT_HERSHEY_SIMPLEX
bottomLeftCornerOfText = (10, 500)
fontScale = 1
fontColor = (255, 255, 255)
thickness = 1
lineType = 2

counter = time.time()


def plot(destination, report=None):

    image = np.frombuffer(destination, dtype=np.dtype("uint8"))
    resized_image = np.reshape(image, (display_width, display_height, 3))
    global counter
    now = time.time()
    cv2.putText(
        resized_image,
        f"Hello {1 / (now - counter):.2f}",
        bottomLeftCornerOfText,
        font,
        fontScale,
        fontColor,
        thickness,
        lineType,
    )
    data = resized_image[:, :, (2, 1, 0)]

    global mutex
    mutex.acquire()
    counter = now
    pygame.surfarray.blit_array(gameDisplay, data)
    pygame.display.flip()
    mutex.release()

    return {"report": bytes("ok", "utf-8")}
