"""TODO: Add docstring."""

import json
import os

import numpy as np
import pyarrow as pa
from dora import Node

node = Node()

IMAGE_RESIZE_RATIO = float(os.getenv("IMAGE_RESIZE_RATIO", "1.0"))


l_init_pose = [
    -7.0631310641087435,
    -10.432298603362307,
    24.429809104404114,
    -132.15000828778648,
    -1.5494749438811133,
    -21.749917789205202,
    8.099312596108344,
    100,
]
r_init_pose = [
    -5.60273587426976,
    10.780818397272316,
    -27.868146823156042,
    -126.15650363072193,
    3.961108018106834,
    -35.43682799906162,
    350.9236448374495,
    100,
]
r_release_closed_pose = [
    -26.1507947940993,
    12.16735021387949,
    -2.2657319092611976,
    -97.63648867582175,
    -19.91084837404425,
    22.10184328619011,
    366.71351223614494,
    0,
]

r_release_opened_pose = [
    -26.1507947940993,
    12.16735021387949,
    -2.2657319092611976,
    -97.63648867582175,
    -19.91084837404425,
    22.10184328619011,
    366.71351223614494,
    100,
]

l_release_opened_pose = [
    -30.04330081906935,
    -7.415231584691132,
    3.6972339048071468,
    -97.7274736257555,
    12.996718740452982,
    30.838020649757016,
    -1.5572310505704858,
    0,
]

l_release_closed_pose = [
    -30.04330081906935,
    -7.415231584691132,
    3.6972339048071468,
    -97.7274736257555,
    12.996718740452982,
    30.838020649757016,
    -1.5572310505704858,
    100,
]


def wait_for_event(id, timeout=None, cache={}):
    """TODO: Add docstring."""
    while True:
        event = node.next(timeout=timeout)
        if event is None:
            cache["finished"] = True
            return None, cache
        if event["type"] == "INPUT":
            cache[event["id"]] = event["value"]
            if event["id"] == id:
                return event["value"], cache

        elif event["type"] == "ERROR":
            return None, cache


arm_holding_object = None
cache = {}


## ---- INIT ---
node.send_output(
    "action_r_arm",
    pa.array(r_init_pose),
    metadata={"encoding": "jointstate", "duration": 2},
)
node.send_output(
    "action_l_arm",
    pa.array(l_init_pose),
    metadata={"encoding": "jointstate", "duration": 2},
)
node.send_output("look", pa.array([0.35, 0, 0]))


for event in node:
    if event["type"] == "INPUT":
        if event["id"] == "pose":
            values = event["value"]
            values = values.to_numpy()
            print("Pose: ", values)
            if len(values) == 0:
                continue
            x = values[0]
            y = values[1]
            z = values[2]
            action = event["metadata"]["action"]

            match action:
                case "grab":
                    if len(values) == 0:
                        continue
                    x = x + 0.01

                    ## Clip the Maximum and minim values for the height of the arm to avoid collision or weird movement.
                    trajectory = np.array(
                        [
                            [x, y, -0.16, 0, 0, 0, 100],
                            [x, y, z, 0, 0, 0, 0],
                            [x, y, -0.16, 0, 0, 0, 0],
                        ],
                    ).ravel()

                    if y < 0:
                        node.send_output(
                            "action_r_arm",
                            pa.array(trajectory),
                            metadata={"encoding": "xyzrpy", "duration": "0.75"},
                        )
                        event = wait_for_event(id="response_r_arm", timeout=5)
                        if event is not None and event[0]:
                            print("Success")
                            arm_holding_object = "right"
                            node.send_output(
                                "action_r_arm",
                                pa.array([0.1, -0.2, -0.16, 0, 0, 0, 0]),
                                metadata={"encoding": "xyzrpy", "duration": "1"},
                            )
                        else:
                            print("Failed: x: ", x, " y: ", y, " z: ", z)
                            node.send_output(
                                "action_r_arm",
                                pa.array(r_init_pose),
                                metadata={"encoding": "jointstate", "duration": "1"},
                            )
                            event = wait_for_event(id="response_r_arm")
                    else:
                        y += 0.03
                        node.send_output(
                            "action_l_arm",
                            pa.array(trajectory),
                            metadata={"encoding": "xyzrpy", "duration": "0.75"},
                        )
                        event = wait_for_event(id="response_l_arm", timeout=5)
                        if event is not None and event[0]:
                            print("Success")
                            arm_holding_object = "left"
                            node.send_output(
                                "action_l_arm",
                                pa.array([0.1, 0.2, -0.16, 0, 0, 0, 0]),
                                metadata={"encoding": "xyzrpy", "duration": "1"},
                            )
                        else:
                            print("Failed")
                            node.send_output(
                                "action_l_arm",
                                pa.array(l_init_pose),
                                metadata={"encoding": "jointstate", "duration": "1"},
                            )
                            event = wait_for_event(id="response_l_arm")
                case "release":
                    if len(values) == 0:
                        continue
                    x = x + 0.01
                    if z < -0.4:
                        z = -0.16

                    ## Clip the Maximum and minim values for the height of the arm to avoid collision or weird movement.
                    trajectory = np.array(
                        [
                            [x, y, z + 0.1, 0, 0, 0, 100],
                        ],
                    ).ravel()

                    if arm_holding_object is None:
                        continue
                    elif arm_holding_object == "right":
                        node.send_output(
                            "action_r_arm",
                            pa.array(trajectory),
                            metadata={"encoding": "xyzrpy", "duration": "0.75"},
                        )
                        event = wait_for_event(id="response_r_arm", timeout=5)
                        if event is not None and event[0]:
                            print("Success release right with", event[0])
                            arm_holding_object = "right"
                            node.send_output(
                                "action_r_arm",
                                pa.array(r_init_pose),
                                metadata={"encoding": "jointstate", "duration": 1},
                            )
                            arm_holding_object = None
                        else:
                            print("Failed: x: ", x, " y: ", y, " z: ", z)
                            node.send_output(
                                "action_r_arm",
                                pa.array(r_init_pose),
                                metadata={"encoding": "jointstate", "duration": "1"},
                            )
                            event = wait_for_event(id="response_r_arm")
                    else:
                        y += 0.03
                        node.send_output(
                            "action_l_arm",
                            pa.array(trajectory),
                            metadata={"encoding": "xyzrpy", "duration": "0.75"},
                        )
                        event = wait_for_event(id="response_l_arm", timeout=5)
                        if event is not None and event[0]:
                            print("Success release left with", event[0])
                            arm_holding_object = "left"
                            node.send_output(
                                "action_l_arm",
                                pa.array(l_init_pose),
                                metadata={"encoding": "jointstate", "duration": 1},
                            )
                            arm_holding_object = None
                        else:
                            print("Failed")
                            node.send_output(
                                "action_l_arm",
                                pa.array(l_init_pose),
                                metadata={"encoding": "jointstate", "duration": "1"},
                            )
                            event = wait_for_event(id="response_l_arm")

        elif event["id"] == "release_right":
            node.send_output(
                "action_r_arm",
                pa.array(
                    [
                        0.4,
                        0,
                        -0.16,
                        0,
                        0,
                        0,
                        100,
                    ],
                ),
                metadata={"encoding": "xyzrpy", "duration": "0.75"},
            )
            event, cache = wait_for_event(id="response_r_arm", cache=cache)
            if event is not None and event[0]:
                node.send_output(
                    "action_r_arm",
                    pa.array(r_init_pose),
                    metadata={"encoding": "jointstate", "duration": 1},
                )
        elif event["id"] == "release_left":
            node.send_output(
                "action_l_arm",
                pa.array(
                    [
                        0.4,
                        0,
                        -0.16,
                        0,
                        0,
                        0,
                        100,
                    ],
                ),
                metadata={"encoding": "xyzrpy", "duration": "0.75"},
            )
            event, cache = wait_for_event(id="response_l_arm", cache=cache)
            if event is not None and event[0]:
                node.send_output(
                    "action_l_arm",
                    pa.array(l_init_pose),
                    metadata={"encoding": "jointstate", "duration": 1},
                )
