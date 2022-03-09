import os
import random
import time

import cv2
import matplotlib.pyplot as plt
import numpy as np
import tensorflow as tf
from lanenet_model import lanenet, lanenet_postprocess

from dora import register

PYLOT_HOME = os.environ["PYLOT_HOME"]
LANENET_MODEL_PATH = (
    PYLOT_HOME
    + "/dependencies/models/lane_detection/lanenet/carla_town_1_2/tusimple"
)

LANE_NUMBER = os.environ["LANE_NUMBER"]


tf.compat.v1.disable_eager_execution()


class LanePredictor:
    def __init__(self, weights):
        """Initializes a LanePredictor which is used to register a callback
        for the RGB images and predict lanes.
        Args:
            weights: The path of the weights to be used in the prediction.
            config: The config to be used for tensorflow.
        """
        self._input_tensor = tf.compat.v1.placeholder(
            dtype=tf.float32, shape=[1, 256, 512, 3], name="input_tensor"
        )
        net = lanenet.LaneNet(phase="test")
        self._binary_seg_ret, self._instance_seg_ret = net.inference(
            input_tensor=self._input_tensor, name="LaneNet"
        )
        self._gpu_options = tf.compat.v1.GPUOptions(
            allow_growth=True,
            per_process_gpu_memory_fraction=0.5,
            allocator_type="BFC",
        )
        self._tf_session = tf.compat.v1.Session(
            config=tf.compat.v1.ConfigProto(
                gpu_options=self._gpu_options, allow_soft_placement=True
            )
        )
        with tf.compat.v1.variable_scope(name_or_scope="moving_avg"):
            variable_averages = tf.train.ExponentialMovingAverage(0.9995)
            variables_to_restore = variable_averages.variables_to_restore()

        self._postprocessor = lanenet_postprocess.LaneNetPostProcessor()
        saver = tf.compat.v1.train.Saver(variables_to_restore)
        with self._tf_session.as_default():
            saver.restore(sess=self._tf_session, save_path=LANENET_MODEL_PATH)

    def process_images(self, msg):
        """This function runs the LaneNet model on each of the image retrieved
        from the simulator.
        Args:
            msg: The RGB image received from the camera.
        """
        # Convert the BGRA image to BGR.
        image = np.frombuffer(msg, dtype=np.dtype("uint8"))
        image = np.reshape(image, (720, 1280, 4))[:, :, :3]
        image_vis = image
        image = cv2.resize(image, (512, 256), interpolation=cv2.INTER_LINEAR)
        resized_image = image
        image = image / 127.5 - 1.0

        binary_seg_image, instance_seg_image = self._tf_session.run(
            [self._binary_seg_ret, self._instance_seg_ret],
            feed_dict={self._input_tensor: [image]},
        )

        postprocess_result = self._postprocessor.postprocess(
            binary_seg_result=binary_seg_image[0],
            instance_seg_result=instance_seg_image[0],
            source_image=image_vis,
        )
        mask_image = postprocess_result["mask_image"]

        return resized_image.tobytes()


lane_operator = LanePredictor(LANENET_MODEL_PATH)


@register(LANE_NUMBER)
def detect_lane(state, change):

    value = bytes(change.value.decode())
    print("recieved data")
    return lane_operator.process_images(value)


if __name__ == "__main__":
    while True:
        time.sleep(5)
