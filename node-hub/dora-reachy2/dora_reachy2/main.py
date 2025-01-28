import os

import time
import numpy as np
import pyarrow as pa
from dora import Node
from reachy2_sdk import ReachySDK
from reachy2_sdk.media.camera import CameraView

default_pos = [
    0,
    -10,
    10,
    0,
    3,
    0,
    0,
]
grab_pos = [
    -35.622223238797865,
    -20.328451262540707,
    -0.8628232733758905,
    -79.19811297361636,
    18.120519188800507,
    21.44259513504777,
    -13.68669505889756,
]
middle_pos = [
    15.006856219792429,
    -11.214779588830954,
    -23.404787247734667,
    -128.51209927167594,
    4.3146467203963335,
    30.852459669662423,
    -4.7167604474024944,
]
fistbump_pos1 = [
    -29.35479384506997,
    -15.984066752271785,
    0.4991847490163521,
    -92.0587376335359,
    -11.329155375351048,
    12.616141453673517,
    33.45114515827176,
]
fistbump_pos2 = [
    6.312824111612264,
    -16.698436237563417,
    -18.66457619959058,
    -132.64037488905836,
    -2.9768602563482487,
    16.127705630517152,
    9.765707145088559,
]
hand_wave_pos1 = [
    -34.9845055514667,
    -4.256103461472862,
    37.1917528987426,
    -132.63047111476183,
    42.11556113932392,
    5.328614342780937,
    28.77740038848836,
]
hand_wave_pos2 = [
    -39.50665085752708,
    4.08569792564386,
    -17.53783575070672,
    -132.6317825110825,
    -17.97474587643951,
    -3.758287258250598,
    -31.217505960042924,
]
hand_shake_pos1 = [
    -11.622430037933055,
    -15.886927801895864,
    -17.790286372619562,
    -114.61062891432732,
    18.956187711109692,
    41.6643315219962,
    -19.462564275796097,
]
hand_shake_pos2 = [
    -13.537550194407517,
    -16.422206312055177,
    -16.635128921693862,
    -49.616830055668764,
    28.267470417886255,
    -15.226397282736855,
    14.290984946657037,
]


def main():
    ROBOT_IP = os.getenv("ROBOT_IP", "172.17.134.85")

    reachy = ReachySDK(ROBOT_IP)

    # reachy.turn_on()
    if reachy.mobile_base is not None:
        reachy.mobile_base.turn_on()

    if reachy.head is not None:
        reachy.head.turn_on()
        reachy.head.goto([0, 0, 0])

    params = reachy.cameras.depth.get_parameters(CameraView.DEPTH)

    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "tick":
                (image_left, _) = reachy.cameras.teleop.get_frame(view=CameraView.LEFT)

                if image_left is int:
                    continue

                node.send_output(
                    "image_left",
                    pa.array(image_left.ravel()),
                    metadata={
                        "encoding": "bgr8",
                        "width": image_left.shape[1],
                        "height": image_left.shape[0],
                    },
                )

                (image_right, _) = reachy.cameras.teleop.get_frame(
                    view=CameraView.RIGHT
                )

                if image_right is int:
                    continue

                node.send_output(
                    "image_right",
                    pa.array(image_right.ravel()),
                    metadata={
                        "encoding": "bgr8",
                        "width": image_right.shape[1],
                        "height": image_right.shape[0],
                    },
                )

                (depth_image, _) = reachy.cameras.depth.get_frame()
                node.send_output(
                    "image_depth",
                    pa.array(depth_image.ravel()),
                    metadata={
                        "encoding": "bgr8",
                        "width": depth_image.shape[1],
                        "height": depth_image.shape[0],
                    },
                )

                (depth_frame, _) = reachy.cameras.depth.get_depth_frame()

                if params is not None and depth_frame is not None:
                    height, width, _distortion_model, _D, K, _R, _P = params
                    depth_frame = depth_frame.ravel().astype(np.float64) / 1_000.0

                    node.send_output(
                        "depth",
                        pa.array(depth_frame),
                        metadata={
                            "width": width,
                            "height": height,
                            "focal": [int(K[0, 0]), int(K[1, 1])],
                            "resolution": [int(K[0, 2]), int(K[1, 2])],
                        },
                    )

                position = reachy.mobile_base.get_current_odometry()
                position = [
                    position["x"],
                    position["y"],
                    0,
                    0,
                    0,
                    position["theta"],
                ]
                node.send_output("position", pa.array(position), metadata={})

            elif event["id"] == "action_base":
                [x, y, _z, _rx, _ry, rz] = event["value"].to_numpy()
                reachy.mobile_base.rotate_by(np.rad2deg(rz))
                reachy.mobile_base.translate_by(x, y)
            elif event["id"] == "action_arm":
                text = event["value"][0].as_py()
                if reachy.l_arm.is_off():
                    reachy.l_arm.turn_on()
                    reachy.l_arm.gripper.turn_on()
                if len(reachy.l_arm.get_goto_queue()) == 0:
                    if text == "grab":
                        if reachy.l_arm.gripper.get_current_opening() > 99:
                            reachy.l_arm.gripper.close()
                        reachy.l_arm.goto(middle_pos)
                        time.sleep(1)
                        reachy.l_arm.gripper.open()
                        reachy.l_arm.goto(grab_pos)
                        time.sleep(3)
                        reachy.l_arm.gripper.close()
                        time.sleep(1)
                        reachy.l_arm.goto(middle_pos)
                        reachy.l_arm.goto(default_pos)
                    if text == "release":
                        if reachy.l_arm.gripper.get_current_opening() > 99:
                            reachy.l_arm.gripper.close()
                        reachy.l_arm.goto(middle_pos)
                        reachy.l_arm.goto(grab_pos)
                        time.sleep(3.5)
                        reachy.l_arm.gripper.open()
                        reachy.l_arm.goto(middle_pos)
                        reachy.l_arm.goto(default_pos)
                    if text == "handwave":
                        if reachy.l_arm.gripper.get_current_opening() < 99:
                            reachy.l_arm.gripper.open()
                        reachy.l_arm.goto(hand_wave_pos1)
                        for i in range(2):
                            reachy.l_arm.goto(hand_wave_pos2, duration=1)
                            reachy.l_arm.goto(hand_wave_pos1, duration=1)
                        reachy.l_arm.goto(default_pos)
                    if text == "fistbump":
                        if reachy.l_arm.gripper.get_current_opening() > 1:
                            reachy.l_arm.gripper.close()
                        reachy.l_arm.goto(fistbump_pos2)
                        reachy.l_arm.goto(fistbump_pos1, duration=1.5)
                        reachy.l_arm.goto(fistbump_pos2, duration=1.5)
                        reachy.l_arm.goto(default_pos)

                    if text == "handshake":  # changer la distance
                        if reachy.l_arm.is_off():
                            reachy.l_arm.turn_on()
                            reachy.l_arm.gripper.turn_on()
                        if len(reachy.l_arm.get_goto_queue()) == 0:
                            if reachy.l_arm.gripper.get_current_opening() < 99:
                                reachy.l_arm.gripper.open()
                            reachy.l_arm.goto(hand_shake_pos1)
                            time.sleep(2)
                            reachy.l_arm.gripper.set_opening(30)
                            reachy.l_arm.goto(hand_shake_pos2, duration=1)
                            reachy.l_arm.goto(hand_shake_pos1, duration=1)
                            time.sleep(2)
                            reachy.l_arm.gripper.set_opening(60)
                            reachy.l_arm.goto(default_pos)
                    if text == "wait":
                        pass

    if reachy.mobile_base is not None:
        reachy.mobile_base.turn_off()

    if reachy.head is not None:
        reachy.head.turn_off()
    reachy.r_arm.turn_off_smoothly()
    reachy.l_arm.turn_off_smoothly()


if __name__ == "__main__":
    main()
