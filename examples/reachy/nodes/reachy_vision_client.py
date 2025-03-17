import argparse
import os
import time
from pathlib import Path

import numpy as np
import pyarrow as pa
from dora import Node
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

# import h5py
from pollen_vision.camera_wrappers.depthai import SDKWrapper
from pollen_vision.camera_wrappers.depthai.utils import get_config_file_path

freq = 30

cam_name = "cam_trunk"

time.sleep(5)
cam = SDKWrapper(get_config_file_path("CONFIG_SR"), fps=freq)
# ret, image = cap.read()

import cv2
import numpy as np

episode = 1
dataset = LeRobotDataset("cadene/reachy2_teleop_remi")
from_index = dataset.episode_data_index["from"][episode]
to_index = dataset.episode_data_index["to"][episode]
actions = dataset.hf_dataset["action"][from_index:to_index]
states = dataset.hf_dataset["observation.state"][from_index:to_index]

images = dataset[0]["observation.images.cam_trunk"]


## Convert image from chw to hwc

# cv2.imwrite("test.jpg", (images.permute((1, 2, 0)).numpy() * 255).astype(np.uint8))
images = images.permute((1, 2, 0)).numpy() * 255
images = images.astype(np.uint8)
images = cv2.cvtColor(images, cv2.COLOR_RGB2BGR)

# start = time.time()
import PIL
import torch

# frame_hwc = (images.permute((1, 2, 0)) * 255).type(torch.uint8).cpu().numpy()

# PIL.Image.fromarray(frame_hwc).save( f"frame_.png")

node = Node()
index = 0


for event in node:
    # import cv2

    # while True:
    id = event["id"]
    cam_data, _, _ = cam.get_data()

    left_rgb = cam_data["left"]
    # cv2.imshow("frame", left_rgb)
    # if cv2.waitKey(1) & 0xFF == ord("q"):
    # images = dataset[from_index.item() + index]["observation.images.cam_trunk"]
    # images = images.numpy().transpose() * 255
    # images = images.astype(np.uint8)
    # break
    # index += 1

    # Convert image to BGR from RGB
    left_rgb = cv2.cvtColor(left_rgb, cv2.COLOR_BGR2RGB)
    node.send_output("cam_trunk", pa.array(left_rgb.ravel()))
