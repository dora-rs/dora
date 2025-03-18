"""TODO: Add docstring."""

import os
import time

import cv2
import numpy as np
import pyarrow as pa
import pyrealsense2 as rs
from dora import Node

RUNNER_CI = True if os.getenv("CI") == "true" else False


def main():
    """TODO: Add docstring."""
    flip = os.getenv("FLIP", "")
    device_serial = os.getenv("DEVICE_SERIAL", "")
    image_height = int(os.getenv("IMAGE_HEIGHT", "480"))
    image_width = int(os.getenv("IMAGE_WIDTH", "640"))
    encoding = os.getenv("ENCODING", "rgb8")
    ctx = rs.context()
    devices = ctx.query_devices()
    if devices.size() == 0:
        raise ConnectionError("No realsense camera connected.")

    # Serial list
    serials = [device.get_info(rs.camera_info.serial_number) for device in devices]
    if device_serial and (device_serial in serials):
        raise ConnectionError(
            f"Device with serial {device_serial} not found within: {serials}.",
        )

    pipeline = rs.pipeline()

    config = rs.config()
    config.enable_device(device_serial)
    config.enable_stream(rs.stream.color, image_width, image_height, rs.format.rgb8, 30)
    config.enable_stream(rs.stream.depth, image_width, image_height)

    align_to = rs.stream.color
    align = rs.align(align_to)

    profile = pipeline.start(config)

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    # rgb_profile = profile.get_stream(rs.stream.color)
    # depth_profile = profile.get_stream(rs.stream.depth)
    # rgb_intr = rgb_profile.as_video_stream_profile().get_intrinsics()
    # rgb_intr = depth_profile.get_extrinsics_to(rgb_profile)
    node = Node()
    start_time = time.time()

    pa.array([])  # initialize pyarrow array

    for event in node:
        # Run this example in the CI for 10 seconds only.
        if RUNNER_CI and time.time() - start_time > 10:
            break

        event_type = event["type"]

        if event_type == "INPUT":
            event_id = event["id"]

            if event_id == "tick":
                frames = pipeline.wait_for_frames()
                aligned_frames = align.process(frames)

                aligned_depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                if not aligned_depth_frame or not color_frame:
                    continue

                depth_image = np.asanyarray(aligned_depth_frame.get_data())
                scaled_depth_image = depth_image * depth_scale
                frame = np.asanyarray(color_frame.get_data())

                ## Change rgb to bgr

                if flip == "VERTICAL":
                    frame = cv2.flip(frame, 0)
                elif flip == "HORIZONTAL":
                    frame = cv2.flip(frame, 1)
                elif flip == "BOTH":
                    frame = cv2.flip(frame, -1)

                # resize the frame
                if (
                    image_width is not None
                    and image_height is not None
                    and (
                        frame.shape[1] != image_width or frame.shape[0] != image_height
                    )
                ):
                    frame = cv2.resize(frame, (image_width, image_height))

                metadata = event["metadata"]
                metadata["encoding"] = encoding
                metadata["width"] = int(frame.shape[1])
                metadata["height"] = int(frame.shape[0])

                # Get the right encoding
                if encoding == "bgr8":
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                elif encoding in ["jpeg", "jpg", "jpe", "bmp", "webp", "png"]:
                    ret, frame = cv2.imencode("." + encoding, frame)
                    if not ret:
                        print("Error encoding image...")
                        continue

                storage = pa.array(frame.ravel())

                # metadata["resolution"] = [int(rgb_intr.width), int(rgb_intr.height)]
                # metadata["focal_length"] = [int(rgb_intr.fx), int(rgb_intr.fy)]
                # metadata["principal_point"] = [int(rgb_intr.ppx), int(rgb_intr.ppy)]
                node.send_output("image", storage, metadata)
                node.send_output(
                    "depth", pa.array(scaled_depth_image.ravel()), metadata,
                )

        elif event_type == "ERROR":
            raise RuntimeError(event["error"])


if __name__ == "__main__":
    main()
