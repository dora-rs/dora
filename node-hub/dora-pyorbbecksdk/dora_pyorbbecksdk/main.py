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
import cv2
import os

try:

    from pyorbbecsdk import Context
except ImportError as err:
    print(
        "Please install pyorbbecsdk first by following the instruction at: https://github.com/orbbec/pyorbbecsdk"
    )
    raise err
from pyorbbecsdk import Config
from pyorbbecsdk import OBError
from pyorbbecsdk import OBSensorType, OBFormat
from pyorbbecsdk import Pipeline, FrameSet
from pyorbbecsdk import VideoStreamProfile
from utils import frame_to_bgr_image
from dora import Node
import pyarrow as pa

ESC_KEY = 27

DEVICE_INDEX = os.getenv("DEVICE_INDEX", 0)


def main():
    node = Node()
    config = Config()
    ctx = Context()
    device_list = ctx.query_devices()
    device = device_list.get_device_by_index(int(DEVICE_INDEX))
    pipeline = Pipeline(device)
    try:
        profile_list = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        try:
            color_profile: VideoStreamProfile = profile_list.get_video_stream_profile(
                640, 0, OBFormat.RGB, 30
            )
        except OBError as e:
            print(e)
            color_profile = profile_list.get_default_video_stream_profile()
            print("color profile: ", color_profile)
        config.enable_stream(color_profile)
    except Exception as e:
        print(e)
        return
    pipeline.start(config)
    for event in node:
        try:
            frames: FrameSet = pipeline.wait_for_frames(100)
            if frames is None:
                continue
            color_frame = frames.get_color_frame()
            if color_frame is None:
                continue
            # covert to RGB format
            color_image = frame_to_bgr_image(color_frame)
            if color_image is None:
                print("failed to convert frame to image")
                continue
            ret, frame = cv2.imencode("." + "jpeg", color_image)
            if ret:
                node.send_output("image", pa.array(frame), {"encoding": "jpeg"})
        except KeyboardInterrupt:
            break
    pipeline.stop()


if __name__ == "__main__":
    main()
