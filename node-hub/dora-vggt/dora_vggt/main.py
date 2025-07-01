"""TODO: Add docstring."""
import io
import os
from collections import deque

import cv2
import numpy as np
import pyarrow as pa
import torch
from dora import Node
from PIL import Image
from vggt.models.vggt import VGGT
from vggt.utils.load_fn import load_and_preprocess_images
from vggt.utils.pose_enc import pose_encoding_to_extri_intri

SCALE_FACTOR = float(os.getenv("SCALE_FACTOR", "1"))
VGGT_NUM_IMAGES = int(os.getenv("VGGT_NUM_IMAGES", "2"))
# bfloat16 is supported on Ampere GPUs (Compute Capability 8.0+)

dtype = torch.bfloat16

# Initialize the model and load the pretrained weights.
# This will automatically download the model weights the first time it's run, which may take a while.
model = VGGT.from_pretrained("facebook/VGGT-1B").to("cuda")
model.eval()

# Import vecdeque


def main():
    """TODO: Add docstring."""
    node = Node()
    raw_images = deque(maxlen=VGGT_NUM_IMAGES)

    for event in node:
        if event["type"] == "INPUT":

            if "image" in event["id"]:
                storage = event["value"]
                metadata = event["metadata"]
                encoding = metadata["encoding"]
                width = metadata["width"]
                height = metadata["height"]

                if (
                    encoding == "bgr8"
                    or encoding == "rgb8"
                    or encoding in ["jpeg", "jpg", "jpe", "bmp", "webp", "png"]
                ):
                    channels = 3
                    storage_type = np.uint8
                else:
                    raise RuntimeError(f"Unsupported image encoding: {encoding}")

                if encoding == "bgr8":
                    frame = (
                        storage.to_numpy()
                        .astype(storage_type)
                        .reshape((height, width, channels))
                    )
                    frame = frame[:, :, ::-1]  # OpenCV image (BGR to RGB)
                elif encoding == "rgb8":
                    frame = (
                        storage.to_numpy()
                        .astype(storage_type)
                        .reshape((height, width, channels))
                    )
                elif encoding in ["jpeg", "jpg", "jpe", "bmp", "webp", "png"]:
                    storage = storage.to_numpy()
                    frame = cv2.imdecode(storage, cv2.IMREAD_COLOR)
                    frame = frame[:, :, ::-1]  # OpenCV image (BGR to RGB)
                else:
                    raise RuntimeError(f"Unsupported image encoding: {encoding}")
                image = Image.fromarray(frame)

                # Save the image to a bytes buffer
                buffer = io.BytesIO()
                image.save(buffer, format="JPEG")  # or JPEG, etc.

                # Rewind the buffer's file pointer to the beginning
                buffer.seek(0)
                raw_images.append(buffer)

                with torch.no_grad():
                    images = load_and_preprocess_images(raw_images).to("cuda")

                    images = images[None]  # add batch dimension
                    aggregated_tokens_list, ps_idx = model.aggregator(images)
                    # Predict Cameras
                    pose_enc = model.camera_head(aggregated_tokens_list)[-1]
                    # Extrinsic and intrinsic matrices, following OpenCV convention (camera from world)
                    extrinsic, intrinsic = pose_encoding_to_extri_intri(
                        pose_enc, images.shape[-2:],
                    )
                    intrinsic = intrinsic[-1][-1]
                    f_0 = intrinsic[0, 0]
                    f_1 = intrinsic[1, 1]
                    r_0 = intrinsic[0, 2]
                    r_1 = intrinsic[1, 2]

                    # Predict Depth Maps
                    depth_map, depth_conf = model.depth_head(
                        aggregated_tokens_list, images, ps_idx,
                    )
                    depth_map[depth_conf < 1.0] = 0.0  # Set low confidence pixels to 0
                    depth_map = depth_map.to(torch.float64)

                    depth_map = depth_map[-1][-1].cpu().numpy()
                    depth_map = SCALE_FACTOR * depth_map
                    # Warning: Make sure to add my_output_id and my_input_id within the dataflow.
                    node.send_output(
                        output_id=event["id"].replace("image", "depth"),
                        data=pa.array(depth_map.ravel()),
                        metadata={
                            "width": depth_map.shape[1],
                            "height": depth_map.shape[0],
                        "focal": [
                            int(f_0),
                            int(f_1),
                        ],
                        "resolution": [
                            int(r_0),
                            int(r_1),
                        ],
                        },
                    )

                    image = images[-1][-1].cpu().numpy() * 255
                    image = image.astype(np.uint8)
                    # reorder pixels to be in last dimension
                    image = image.transpose(1, 2, 0)

                    # Warning: Make sure to add my_output_id and my_input_id within the dataflow.
                    node.send_output(
                        output_id=event["id"],
                        data=pa.array(image.ravel()),
                        metadata={
                            "encoding": "rgb8",
                            "width": image.shape[1],
                            "height": image.shape[0],
                        },
                    )


if __name__ == "__main__":
    main()
