import os
import time

import cv2
import numpy as np
import tensorflow as tf

from dora import register

SRC_LABELS = os.environ["SRC_LABELS"]
DST_LABELS = os.environ["OP_LABELS"]
TRAFFIC_LIGHT_DET_MIN_SCORE_THRESHOLD = 0.01
WIDTH = 1043
HEIGHT = 587
GPU_DEVICE = 0
TRAFFIC_LIGHT_MODEL_PATH = (
    os.environ["PYLOT_HOME"]
    + "/dependencies/models/traffic_light_detection/faster-rcnn"
)
physical_devices = tf.config.experimental.list_physical_devices("GPU")
tf.config.experimental.set_visible_devices(physical_devices[GPU_DEVICE], "GPU")
tf.config.experimental.set_memory_growth(physical_devices[GPU_DEVICE], True)

# Load the model from the saved_model format file.
model = tf.saved_model.load(TRAFFIC_LIGHT_MODEL_PATH)


@register(SRC_LABELS, DST_LABELS)
def operator(state, change):
    """
    Publication of change
    """
    value = bytes(change.value.get_content())
    array = np.frombuffer(value, dtype=np.dtype("uint8"))
    array = array.reshape((587, 1043, 3))
    image_np_expanded = np.expand_dims(array, axis=0)

    infer = model.signatures["serving_default"]
    result = infer(tf.convert_to_tensor(value=image_np_expanded))

    boxes = result["boxes"]
    scores = result["scores"]
    classes = result["classes"]
    num_detections = result["detections"]

    num_detections = int(num_detections[0])
    boxes = boxes[0][:num_detections]
    scores = scores[0][:num_detections]

    for index in range(len(scores)):
        if scores[index] > TRAFFIC_LIGHT_DET_MIN_SCORE_THRESHOLD:

            # Add the patch to the Axes
            cv2.rectangle(
                array,
                (
                    int(boxes[index][1] * WIDTH),
                    int(boxes[index][0] * HEIGHT),
                ),
                (
                    int(boxes[index][3] * WIDTH),
                    int(boxes[index][2] * HEIGHT),
                ),
                (255, 0, 0),
            )

    return array.tobytes()


if __name__ == "__main__":
    time.sleep(60)
