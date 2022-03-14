import os
import random
import time

import matplotlib.pyplot as plt
import numpy as np

from dora import register_sink

LABELS = os.environ["NAME"]


data = np.zeros((720, 1280, 3))

import random
import threading
import time

mutex = threading.Lock()

plt.ion()
fig, axes = plt.subplots(1)
plt.show()


@register_sink(LABELS)
def detect_lane(state, change):

    value = bytes(change.value.decode())

    image = np.frombuffer(value, dtype=np.dtype("uint8"))
    resized_image = np.reshape(image, (256, 512, 3))
    print("recieved data 0")
    global mutex
    global data
    mutex.acquire()
    data = resized_image  # [:, :, (2, 1, 0)]
    mutex.release()


if __name__ == "__main__":
    while True:
        plt.pause(0.1)
        axes.imshow(data)
        plt.draw()
    time.sleep(60)
