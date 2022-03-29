import tensorflow as tf
import os
import numpy as np
import cv2
import time


def load_coco_labels(labels_path):
    """Returns a map from index to label.
    Args:
        labels_path (:obj:`str`): Path to a file storing a label on each line.
    """
    labels_map = {}
    with open(labels_path) as labels_file:
        labels = labels_file.read().splitlines()
        index = 1
        for label in labels:
            labels_map[index] = label
            index += 1
    return labels_map


def load_serving_model(model_name, model_path, gpu_memory_fraction):
    detection_graph = tf.Graph()
    with detection_graph.as_default():
        # Load a frozen graph.
        graph_def = tf.compat.v1.GraphDef()
        with tf.io.gfile.GFile(model_path, "rb") as f:
            graph_def.ParseFromString(f.read())
            tf.import_graph_def(graph_def, name="")
    gpu_options = tf.compat.v1.GPUOptions(
        allow_growth=True,
        per_process_gpu_memory_fraction=gpu_memory_fraction,
    )
    return model_name, tf.compat.v1.Session(
        graph=detection_graph,
        config=tf.compat.v1.ConfigProto(gpu_options=gpu_options),
    )


MODEL_PATH = (
    os.environ["PYLOT_HOME"]
    + "/dependencies/models/obstacle_detection/efficientdet/efficientdet-d0/efficientdet-d0_frozen.pb"
)
LABELS_PATH = os.environ["PYLOT_HOME"] + "/dependencies/models/coco.names"

WIDTH = 587
HEIGHT = 1043
OBSTACLE_THRESHOLD = 0.1
coco_labels = load_coco_labels(LABELS_PATH)
models, tf_session = load_serving_model(
    "efficientdet-d0", model_path=MODEL_PATH, gpu_memory_fraction=0.9
)
signitures = {
    "image_files": "image_files:0",
    "image_arrays": "image_arrays:0",
    "prediction": "detections:0",
}


# Serve some junk image to load up the model.
inputs = np.zeros((108, 192, 3), dtype="uint8")
tf_session.run(
    signitures["prediction"],
    feed_dict={signitures["image_arrays"]: [inputs]},
)[0]


def run(image, destination=None):

    image = np.frombuffer(image, dtype=np.dtype("uint8"))
    inputs = np.reshape(image, (587, 1043, 3))

    outputs_np = tf_session.run(
        signitures["prediction"],
        feed_dict={signitures["image_arrays"]: [inputs]},
    )[0]

    obstacles = []

    for _, ymin, xmin, ymax, xmax, score, _class in outputs_np:
        xmin, ymin, xmax, ymax = int(xmin), int(ymin), int(xmax), int(ymax)
        if _class in coco_labels:
            if score >= OBSTACLE_THRESHOLD:
                xmin, xmax = max(0, xmin), min(xmax, WIDTH)
                ymin, ymax = max(0, ymin), min(ymax, HEIGHT)
                if xmin < xmax and ymin < ymax:
                    obstacles.append(
                        (
                            (xmin, xmax, ymin, ymax),
                            score,
                            coco_labels[_class],
                        )
                    )

                    # Add the patch to the Axes
                    cv2.rectangle(
                        inputs,
                        (
                            xmin,
                            ymin,
                        ),
                        (
                            xmax,
                            ymax,
                        ),
                        (255, 0, 0),
                    )

    return {"destination": inputs.tobytes()}
