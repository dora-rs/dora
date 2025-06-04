"""TODO: Add docstring."""

# ******************************************************************************
#  Copyright (c) 2023 Orbbec 3D Technology, Inc
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http:# www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
# ******************************************************************************
import os

import cv2
import numpy as np
import pyarrow as pa
from dora import Node

try:
    from pyorbbecsdk import (
        Config,
        Context,
        FrameSet,
        OBError,
        OBFormat,
        OBSensorType,
        Pipeline,
        VideoFrame,
        VideoStreamProfile,
    )
except ImportError as err:
    print(
        "Please install pyorbbecsdk first by following the instruction at: https://github.com/orbbec/pyorbbecsdk",
    )
    raise err


class TemporalFilter:
    """TODO: Add docstring."""

    def __init__(self, alpha):
        """TODO: Add docstring."""
        self.alpha = alpha
        self.previous_frame = None

    def process(self, frame):
        """TODO: Add docstring."""
        if self.previous_frame is None:
            result = frame
        else:
            result = cv2.addWeighted(
                frame,
                self.alpha,
                self.previous_frame,
                1 - self.alpha,
                0,
            )
        self.previous_frame = result
        return result


def yuyv_to_bgr(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    """TODO: Add docstring."""
    yuyv = frame.reshape((height, width, 2))
    return cv2.cvtColor(yuyv, cv2.COLOR_YUV2BGR_YUY2)


def uyvy_to_bgr(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    """TODO: Add docstring."""
    uyvy = frame.reshape((height, width, 2))
    return cv2.cvtColor(uyvy, cv2.COLOR_YUV2BGR_UYVY)


def i420_to_bgr(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    """TODO: Add docstring."""
    y = frame[0:height, :]
    u = frame[height : height + height // 4].reshape(height // 2, width // 2)
    v = frame[height + height // 4 :].reshape(height // 2, width // 2)
    yuv_image = cv2.merge([y, u, v])
    return cv2.cvtColor(yuv_image, cv2.COLOR_YUV2BGR_I420)


def nv21_to_bgr(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    """TODO: Add docstring."""
    y = frame[0:height, :]
    uv = frame[height : height + height // 2].reshape(height // 2, width)
    yuv_image = cv2.merge([y, uv])
    return cv2.cvtColor(yuv_image, cv2.COLOR_YUV2BGR_NV21)


def nv12_to_bgr(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    """TODO: Add docstring."""
    y = frame[0:height, :]
    uv = frame[height : height + height // 2].reshape(height // 2, width)
    yuv_image = cv2.merge([y, uv])
    return cv2.cvtColor(yuv_image, cv2.COLOR_YUV2BGR_NV12)


def frame_to_bgr_image(frame: VideoFrame):
    """TODO: Add docstring."""
    width = frame.get_width()
    height = frame.get_height()
    color_format = frame.get_format()
    data = np.asanyarray(frame.get_data())
    image = np.zeros((height, width, 3), dtype=np.uint8)
    if color_format == OBFormat.RGB:
        image = np.resize(data, (height, width, 3))
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    elif color_format == OBFormat.BGR:
        image = np.resize(data, (height, width, 3))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    elif color_format == OBFormat.YUYV:
        image = np.resize(data, (height, width, 2))
        image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_YUYV)
    elif color_format == OBFormat.MJPG:
        image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    elif color_format == OBFormat.I420:
        return i420_to_bgr(data, width, height)
    elif color_format == OBFormat.NV12:
        return nv12_to_bgr(data, width, height)
    elif color_format == OBFormat.NV21:
        return nv21_to_bgr(data, width, height)
    elif color_format == OBFormat.UYVY:
        image = np.resize(data, (height, width, 2))
        image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_UYVY)
    else:
        print(f"Unsupported color format: {color_format}")
        return None
    return image


ESC_KEY = 27
MIN_DEPTH_METERS = 0.01
MAX_DEPTH_METERS = 15.0

DEVICE_INDEX = int(os.getenv("DEVICE_INDEX", "0"))


def main():
    """TODO: Add docstring."""
    node = Node()
    config = Config()
    ctx = Context()
    device_list = ctx.query_devices()
    device = device_list.get_device_by_index(int(DEVICE_INDEX))
    temporal_filter = TemporalFilter(alpha=0.5)
    pipeline = Pipeline(device)
    profile_list = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
    try:
        color_profile: VideoStreamProfile = profile_list.get_video_stream_profile(
            640,
            480,
            OBFormat.RGB,
            30,
        )
    except OBError as e:
        print(e)
        color_profile = profile_list.get_default_video_stream_profile()
        print("color profile: ", color_profile)
    profile_list = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
    try:
        depth_profile: VideoStreamProfile = profile_list.get_video_stream_profile(
            640,
            480,
            OBFormat.Y16,
            30,
        )
    except OBError as e:
        print(e)
        depth_profile = profile_list.get_default_video_stream_profile()
        print("depth profile: ", depth_profile)
    config.enable_stream(color_profile)
    config.enable_stream(depth_profile)
    pipeline.start(config)
    for _event in node:
        try:
            frames: FrameSet = pipeline.wait_for_frames(100)
            if frames is None:
                continue

            # Get Color image
            color_frame = frames.get_color_frame()
            if color_frame is None:
                continue
            # convert to RGB format
            color_image = frame_to_bgr_image(color_frame)
            if color_image is None:
                print("failed to convert frame to image")
                continue
            # Send Color Image
            ret, frame = cv2.imencode("." + "jpeg", color_image)
            if ret:
                node.send_output("image", pa.array(frame), {"encoding": "jpeg"})

            # Get Depth data
            depth_frame = frames.get_depth_frame()
            if depth_frame is None:
                continue
            width = depth_frame.get_width()
            height = depth_frame.get_height()
            scale = depth_frame.get_depth_scale()
            depth_data = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)
            depth_data = depth_data.reshape((height, width))
            depth_data = depth_data.astype(np.float32) * scale * 0.001
            depth_data = np.where(
                (depth_data > MIN_DEPTH_METERS) & (depth_data < MAX_DEPTH_METERS),
                depth_data,
                0,
            )
            depth_data = temporal_filter.process(depth_data)
            # Send Depth data
            storage = pa.array(depth_data.ravel())
            node.send_output("depth", storage)
            # Convert to Image
            depth_image = cv2.normalize(
                depth_data,
                None,
                0,
                255,
                cv2.NORM_MINMAX,
                dtype=cv2.CV_8U,
            )
            # Send Depth Image
            depth_image = cv2.applyColorMap(depth_image, cv2.COLORMAP_JET)
            ret, frame = cv2.imencode("." + "jpeg", depth_image)
            if ret:
                node.send_output("image_depth", pa.array(frame), {"encoding": "jpeg"})

        except KeyboardInterrupt:
            break
    pipeline.stop()


if __name__ == "__main__":
    main()
