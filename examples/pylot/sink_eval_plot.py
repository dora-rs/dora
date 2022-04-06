import os
import threading
import time

import cv2
import numpy as np
import pygame

mutex = threading.Lock()
pygame.init()


display_width = 800
display_height = 600

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


def plot(inputs):
    destination = inputs[os.environ["SOURCE"]]
    image = np.frombuffer(destination, dtype=np.dtype("uint8"))
    if len(image) == 800 * 600 * 4:
        resized_image = np.reshape(image, (display_height, display_width, 4))
        resized_image = resized_image[:, :, :3]
        resized_image = np.ascontiguousarray(resized_image, dtype=np.uint8)
    else:
        resized_image = np.reshape(image, (display_height, display_width, 3))
    global counter
    now = time.time()
    cv2.putText(
        resized_image,
        f"Hertz {1 / (now - counter):.2f}",
        bottomLeftCornerOfText,
        font,
        fontScale,
        fontColor,
        thickness,
        lineType,
    )
    data = resized_image[:, :, (2, 1, 0)]
    data = np.rot90(data)

    global mutex
    mutex.acquire()
    counter = now
    pygame.surfarray.blit_array(gameDisplay, data)
    pygame.display.flip()
    mutex.release()

    return {"report": bytes("ok", "utf-8")}
