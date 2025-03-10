"""TODO: Add docstring."""

from threading import Event

import cv2
import numpy as np
import pyarrow as pa
from dora import Node
from record3d import Record3DStream


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
        """This method is called from non-main thread, therefore cannot be used for presenting UI."""
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

    def start_processing_stream(self):
        """TODO: Add docstring."""
        node = Node()

        for event in node:
            if self.stop:
                break
            if event["type"] == "INPUT":
                if event["id"] == "TICK":
                    self.event.wait()  # Wait for new frame to arrive

                    # Copy the newly arrived RGBD frame
                    depth = self.session.get_depth_frame()
                    rgb = self.session.get_rgb_frame()
                    intrinsic_mat = self.get_intrinsic_mat_from_coeffs(
                        self.session.get_intrinsic_mat(),
                    )

                    if depth.shape != rgb.shape:
                        rgb = cv2.resize(rgb, (depth.shape[1], depth.shape[0]))

                    node.send_output(
                        "image",
                        pa.array(rgb.ravel()),
                        metadata={
                            "encoding": "rgb8",
                            "width": rgb.shape[1],
                            "height": rgb.shape[0],
                        },
                    )

                    node.send_output(
                        "depth",
                        pa.array(depth.ravel().astype(np.float64())),
                        metadata={
                            "width": depth.shape[1],
                            "height": depth.shape[0],
                            "encoding": "CV_64F",
                            "focal": [
                                int(intrinsic_mat[0, 0]),
                                int(intrinsic_mat[1, 1]),
                            ],
                            "resolution": [
                                int(intrinsic_mat[0, 2]),
                                int(intrinsic_mat[1, 2]),
                            ],
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
