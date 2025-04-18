"""TODO: Add docstring."""

import os
from threading import Event

import cv2
import numpy as np
import pyarrow as pa
from dora import Node
from record3d import Record3DStream
from scipy.spatial.transform import Rotation

image_width = os.getenv("IMAGE_WIDTH")
image_height = os.getenv("IMAGE_HEIGHT")
ROTATE = os.getenv("ROTATE")


class DemoApp:
    """TODO: Add docstring."""

    def __init__(self):
        """TODO: Add docstring."""
        self.event = Event()
        self.session = None
        self.DEVICE_TYPE__TRUEDEPTH = 0
        self.DEVICE_TYPE__LIDAR = 1
        self.stop = False

    def on_new_frame(self):
        """on_new_frame method is called from non-main thread, therefore cannot be used for presenting UI."""
        self.event.set()  # Notify the main thread to stop waiting and process new frame.

    def on_stream_stopped(self):
        """TODO: Add docstring."""
        self.stop = True
        print("Stream stopped")

    def connect_to_device(self, dev_idx):
        """TODO: Add docstring."""
        print("Searching for devices")
        devs = Record3DStream.get_connected_devices()
        print(f"{len(devs)} device(s) found")
        for dev in devs:
            print(f"\tID: {dev.product_id}\n")

        if len(devs) <= dev_idx:
            raise RuntimeError(
                f"Cannot connect to device #{dev_idx}, try different index.",
            )

        dev = devs[dev_idx]
        self.session = Record3DStream()
        self.session.on_new_frame = self.on_new_frame
        self.session.on_stream_stopped = self.on_stream_stopped
        self.session.connect(dev)  # Initiate connection and start capturing

    def get_intrinsic_mat_from_coeffs(self, coeffs):
        """TODO: Add docstring."""
        return np.array(
            [[coeffs.fx, 0, coeffs.tx], [0, coeffs.fy, coeffs.ty], [0, 0, 1]],
        )

    def get_camera_pose(self):
        """Get Camera Pose."""
        pose = self.session.get_camera_pose()
        rot = Rotation.from_quat([pose.qx, pose.qy, pose.qz, pose.qw])
        euler = rot.as_euler("xyz", degrees=False)

        return [
            pose.tx,
            pose.ty,
            pose.tz,
            pose.qx,
            euler[1],
            euler[2],
        ]

    def start_processing_stream(self):
        """TODO: Add docstring."""
        node = Node()

        for event in node:
            if self.stop:
                break

            if event["type"] == "INPUT":
                self.event.wait()  # Wait for new frame to arrive

                # Copy the newly arrived RGBD frame
                depth = self.session.get_depth_frame()
                rgb = self.session.get_rgb_frame()
                intrinsic_mat = self.get_intrinsic_mat_from_coeffs(
                    self.session.get_intrinsic_mat(),
                )
                # pose = self.get_camera_pose()

                if depth.shape != rgb.shape:
                    rgb = cv2.resize(rgb, (depth.shape[1], depth.shape[0]))
                if image_width is not None and image_height is not None:
                    f_0 = intrinsic_mat[0, 0] * (int(image_height) / rgb.shape[0])
                    f_1 = intrinsic_mat[1, 1] * (int(image_width) / rgb.shape[1])
                    r_0 = intrinsic_mat[0, 2] * (int(image_height) / rgb.shape[0])
                    r_1 = intrinsic_mat[1, 2] * (int(image_width) / rgb.shape[1])
                    if ROTATE == "ROTATE_90_CLOCKWISE":
                        rgb = cv2.rotate(rgb, cv2.ROTATE_90_CLOCKWISE)
                        depth = cv2.rotate(depth, cv2.ROTATE_90_CLOCKWISE)
                    rgb = cv2.resize(rgb, (int(image_width), int(image_height)))
                    depth = cv2.resize(
                        depth,
                        (int(image_width), int(image_height)),
                        interpolation=cv2.INTER_NEAREST,
                    )
                else:
                    f_0 = intrinsic_mat[0, 0]
                    f_1 = intrinsic_mat[1, 1]
                    r_0 = intrinsic_mat[0, 2]
                    r_1 = intrinsic_mat[1, 2]

                node.send_output(
                    "image",
                    pa.array(rgb.ravel()),
                    metadata={
                        "encoding": "rgb8",
                        "width": rgb.shape[1],
                        "height": rgb.shape[0],
                    },
                )

                depth = (np.array(depth) * 1_000).astype(np.uint16)
                depth = np.clip(depth, 0, 4095)  # Clip depth to uint12
                node.send_output(
                    "depth",
                    pa.array(depth.ravel()),
                    metadata={
                        "width": depth.shape[1],
                        "height": depth.shape[0],
                        "encoding": "mono16",
                        "focal": [
                            int(f_0),
                            int(f_1),
                        ],
                        "resolution": [
                            int(r_0),
                            int(r_1),
                        ],
                        # "roll": pose[3],
                        # "pitch": pose[4],  # Adding 90 degrees to pitch
                        # "yaw": pose[5],
                    },
                )

            self.event.clear()


def main():
    """TODO: Add docstring."""
    app = DemoApp()
    app.connect_to_device(dev_idx=0)
    app.start_processing_stream()


if __name__ == "__main__":
    main()
