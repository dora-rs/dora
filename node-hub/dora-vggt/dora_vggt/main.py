"""TODO: Add docstring."""

import io
import math
import os
from collections import deque

import cv2
import numpy as np
import pyarrow as pa
import torch
from dora import Node
from PIL import Image
from vggt.models.vggt import VGGT
from vggt.utils.geometry import unproject_depth_map_to_point_map
from vggt.utils.load_fn import load_and_preprocess_images
from vggt.utils.pose_enc import pose_encoding_to_extri_intri

SCALE_FACTOR = float(os.getenv("SCALE_FACTOR", "1"))
VGGT_NUM_IMAGES = int(os.getenv("VGGT_NUM_IMAGES", "2"))
# bfloat16 is supported on Ampere GPUs (Compute Capability 8.0+)

dtype = torch.bfloat16

# Check if cuda is available and set the device accordingly
device = "cuda" if torch.cuda.is_available() else "cpu"

# Initialize the model and load the pretrained weights.
# This will automatically download the model weights the first time it's run, which may take a while.
model = VGGT.from_pretrained("facebook/VGGT-1B").to(device)
model.eval()

DEPTH_ENCODING = os.environ.get("DEPTH_ENCODING", "float64")
# Import vecdeque


# -------------------------- Convert VGGT point map to SONATA format -----------------------------
def convert_vggt_to_sonata(
    point_map_by_unprojection: np.ndarray, images=not None, conf_threshold=math.inf
):
    """'Convert VGGT point map to Sonata usable format"""

    def normal_from_cross_product(points_2d: np.ndarray) -> np.ndarray:
        dzdy = points_2d[1:, :-1, :] - points_2d[:-1, :-1, :]  # vertical diff
        dzdx = points_2d[:-1, 1:, :] - points_2d[:-1, :-1, :]  # horizontal diff
        normals = np.cross(dzdx, dzdy)
        norms = np.linalg.norm(normals, axis=-1, keepdims=True)
        normals = np.divide(
            normals, norms, out=np.zeros_like(normals), where=norms != 0
        )
        return normals  # [H-1, W-1, 3]

    S, H, W, _ = point_map_by_unprojection.shape  # S, H, W, 3
    H_valid = H - 1
    W_valid = W - 1
    coords_cropped = []
    colors_cropped = []
    normals_list = []

    for s in range(S):
        coords = point_map_by_unprojection[s, :H_valid, :W_valid].reshape(-1, 3)
        coords_cropped.append(coords)

        normals = normal_from_cross_product(
            point_map_by_unprojection[s]
        )  # [H-1, W-1, 3]
        normals_list.append(normals.reshape(-1, 3))  # [(H-1)*(W-1), 3]

        if images is not None:
            img = images[0, s] if images.dim() == 5 else images[s]
            img_np = img.permute(1, 2, 0).cpu().numpy()
            color = img_np[:H_valid, :W_valid].reshape(-1, 3)
            colors_cropped.append(color)

    coords_all = np.concatenate(coords_cropped, axis=0)
    normals_all = np.concatenate(normals_list, axis=0)
    colors_all = np.concatenate(colors_cropped, axis=0)

    # depth mask
    z_values = coords_all[:, 1]
    height_mask = z_values < conf_threshold

    coords_all = coords_all[height_mask]
    normals_all = normals_all[height_mask]
    colors_all = colors_all[height_mask]

    sonata_dict = {
        "coord": torch.from_numpy(coords_all).float(),
        "normal": torch.from_numpy(normals_all).float(),
        "color": torch.from_numpy(colors_all).float(),
    }

    image1 = {
        "coord": coords_cropped[0],
        "normal": normals_list[0],
        "color": colors_cropped[0],
    }

    return sonata_dict, image1


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
                    images = load_and_preprocess_images(raw_images).to(device)

                    images = images[None]  # add batch dimension
                    aggregated_tokens_list, ps_idx = model.aggregator(images)
                    # Predict Cameras
                    pose_enc = model.camera_head(aggregated_tokens_list)[-1]
                    # Extrinsic and intrinsic matrices, following OpenCV convention (camera from world)
                    extrinsic, intrinsic = pose_encoding_to_extri_intri(
                        pose_enc, images.shape[-2:]
                    )  # (B, S, 3, 4) and (B, S, 3, 3)

                    B, V = extrinsic.shape[:2]  # [1, 6, 3, 4]
                    extrinsic_homo = torch.eye(4, device=device).repeat(
                        B, V, 1, 1
                    )  # [1, 6, 4, 4]
                    extrinsic_homo[:, :, :3, :] = extrinsic
                    transformation = torch.tensor(
                        [
                            [1, 0, 0, 0],  # x right -> x right
                            [0, 0, -1, 0],  # y down -> z up
                            [
                                0,
                                -1,
                                0,
                                0,
                            ],  # z forward -> y towards (forward translation)
                            [0, 0, 0, 1],
                        ],
                        dtype=torch.float32,
                        device=extrinsic.device,
                    )

                    transformation = transformation[None, None, :, :]  # [1, 1, 4, 4]
                    extrinsic_homo = extrinsic_homo @ transformation  # [B, S, 4, 4]
                    extrinsic = extrinsic_homo[:, :, :3, :]
                    intrinsic_ = intrinsic[-1][-1]
                    f_0 = intrinsic_[0, 0]
                    f_1 = intrinsic_[1, 1]
                    r_0 = intrinsic_[0, 2]
                    r_1 = intrinsic_[1, 2]

                    # Predict Depth Maps
                    depth_map, depth_conf = model.depth_head(
                        aggregated_tokens_list, images, ps_idx
                    )

                    # point cloud
                    point_map_by_unprojection = unproject_depth_map_to_point_map(
                        depth_map.squeeze(0), extrinsic.squeeze(0), intrinsic.squeeze(0)
                    )
                    Sonata_format, point_cloud_img1 = convert_vggt_to_sonata(
                        point_map_by_unprojection, images=images
                    )

                    depth_map[depth_conf < 1.0] = 0.0  # Set low confidence pixels to 0
                    depth_map = depth_map.to(torch.float64)

                    depth_map = depth_map[-1][-1].cpu().numpy()
                    depth_map = SCALE_FACTOR * depth_map
                    # Warning: Make sure to add my_output_id and my_input_id within the dataflow.
                    if DEPTH_ENCODING == "mono16":
                        depth_map = (depth_map * 1000).astype(np.uint16)

                    node.send_output(
                        output_id=event["id"].replace("image", "depth"),
                        data=pa.array(depth_map.ravel()),
                        metadata={
                            "width": depth_map.shape[1],
                            "height": depth_map.shape[0],
                            "encoding": DEPTH_ENCODING,
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

                    torch.save(Sonata_format, "predictions.pt")


if __name__ == "__main__":
    main()
