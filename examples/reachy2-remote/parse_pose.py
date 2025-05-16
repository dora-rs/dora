"""TODO: Add docstring."""

import os
import numpy as np
import pyarrow as pa
from dora import Node

node = Node()

IMAGE_RESIZE_RATIO = float(os.getenv("IMAGE_RESIZE_RATIO", "1.0"))


l_init_pose = [
    -7.0631310641087435,
    -10.432298603362307,
    57.429809104404114,
    -126.15000828778648,
    -3.5494749438811133,
    -35.749917789205202,
    0.999312596108344,
    100,
]
r_init_pose = [
    -5.60273587426976,
    10.780818397272316,
    -57.868146823156042,
    -126.15650363072193,
    3.961108018106834,
    -35.43682799906162,
    0.9236448374495,
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
node.send_output("look", pa.array([1.0, 0, 0]))


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
            node.send_output("look", pa.array([x, y, z]))

            match action:
                case "pick":
                    if len(values) == 0:
                        continue
                    x = x + 0.05
                    z = z + 0.03

                    if y < 0:  # right arm
                        node.send_output("look", pa.array([x, y, z]))

                        # Send a mobile base command to move slightly left to facilitate the grasp
                        node.send_output(
                            "translate_base",
                            pa.array([0.0, y + 0.3, 0, 0, 0, 0]),
                            metadata={"encoding": "xyzrpy", "duration": "0.3"},
                        )
                        event = wait_for_event(id="translate_base")
                        node.send_output("look", pa.array([x, -0.3, z]))

                        trajectory = np.array(
                            [
                                [x, -0.3, z + 0.1, 0, 0, 0, 100],
                                [x, -0.3, z, 0, 0, 0, 0],
                                [x, -0.3, z + 0.1, 0, 0, 0, 0],
                                [0.3, -0.3, -0.16, 0, 0, 0, 0],
                            ],
                        ).ravel()
                        node.send_output(
                            "action_r_arm",
                            pa.array(trajectory),
                            metadata={"encoding": "xyzrpy", "duration": "0.3"},
                        )
                        event = wait_for_event(id="response_r_arm")

                        node.send_output("look", pa.array([x, -0.3, z]))

                        node.send_output(
                            "translate_base",
                            pa.array([0.0, -(y + 0.3), 0, 0, 0, 0]),
                            metadata={"encoding": "xyzrpy", "duration": "0.3"},
                        )
                        event = wait_for_event(id="translate_base")
                        arm_holding_object = "right"
                        node.send_output("look", pa.array([0.4, 0, 0]))
                        node.send_output("success", pa.array([True]))

                    else:  # left arm
                        # Send a mobile base command to move slightly left to facilitate the grasp
                        node.send_output("look", pa.array([x, y, z]))

                        ## Move the base
                        node.send_output(
                            "translate_base",
                            pa.array([0.0, y - 0.3, 0, 0, 0, 0]),
                            metadata={"encoding": "xyzrpy", "duration": "0.3"},
                        )
                        event = wait_for_event(id="translate_base")

                        node.send_output("look", pa.array([x, 0.3, z]))
                        trajectory = np.array(
                            [
                                [x, 0.3, z + 0.1, 0, 0, 0, 100],
                                [x, 0.3, z, 0, 0, 0, 0],
                                [x, 0.3, z + 0.1, 0, 0, 0, 0],
                                [0.3, 0.3, -0.16, 0, 0, 0, 0],
                            ],
                        ).ravel()
                        node.send_output("look", pa.array([x, 0.3, z]))

                        node.send_output(
                            "action_l_arm",
                            pa.array(trajectory),
                            metadata={"encoding": "xyzrpy", "duration": "0.3"},
                        )

                        event = wait_for_event(id="response_l_arm")
                        if not (
                            event is not None
                            and event[0] is not None
                            and event[0][0].as_py()
                        ):
                            node.send_output(
                            "action_l_arm",
                            pa.array(trajectory),
                            metadata={"encoding": "xyzrpy", "duration": "0.3"},
                        )

                            event = wait_for_event(id="response_l_arm")
                        node.send_output(
                            "translate_base",
                            pa.array([0.0, -(y - 0.3), 0, 0, 0, 0]),
                            metadata={"encoding": "xyzrpy", "duration": "0.3"},
                        )
                        event = wait_for_event(id="translate_base")
                        node.send_output("look", pa.array([0.4, 0.0, 0.0]))

                        arm_holding_object = "left"
                        node.send_output("success", pa.array([True]))
                case "flip":
                    if len(values) == 0:
                        continue
                    x = x + 0.05
                    z = z + 0.03

                    if y < 0:  # right arm
                        node.send_output("look", pa.array([x, y, z]))

                        # Send a mobile base command to move slightly left to facilitate the grasp
                        node.send_output(
                            "translate_base",
                            pa.array([0.0, y + 0.3, 0, 0, 0, 0]),
                            metadata={"encoding": "xyzrpy", "duration": "0.3"},
                        )
                        event = wait_for_event(id="translate_base")
                        node.send_output("look", pa.array([x, -0.3, z]))

                        trajectory = np.array(
                            [
                                [x, -0.3, z + 0.1, 0, 0, 0, 100],
                                [x, -0.3, z, 0, 0, 0, 0],
                                [x, -0.3, z + 0.1, 0, 0, 0, 0],
                                [x, -0.3, z + 0.1, -np.pi, 0, 0, 0],
                                [x, -0.3, z, -np.pi, 0, 0, 100],
                                [x, -0.3, z + 0.1, -np.pi, 0, 0, 100],
                                [x, -0.3, z + 0.1, 0, 0, 0, 100],
                            ],
                        ).ravel()
                        node.send_output(
                            "action_r_arm",
                            pa.array(trajectory),
                            metadata={"encoding": "xyzrpy", "duration": "0.8"},
                        )
                        event = wait_for_event(id="response_r_arm")

                        node.send_output("look", pa.array([x, -0.3, z]))

                        node.send_output(
                            "translate_base",
                            pa.array([0.0, -(y + 0.3), 0, 0, 0, 0]),
                            metadata={"encoding": "xyzrpy", "duration": "0.3"},
                        )
                        event = wait_for_event(id="translate_base")
                        arm_holding_object = "right"
                        node.send_output("look", pa.array([0.4, 0, 0]))
                        node.send_output("success", pa.array([True]))

                    else:  # left arm
                        # Send a mobile base command to move slightly left to facilitate the grasp
                        node.send_output("look", pa.array([x, y, z]))

                        ## Move the base
                        node.send_output(
                            "translate_base",
                            pa.array([0.0, y - 0.3, 0, 0, 0, 0]),
                            metadata={"encoding": "xyzrpy", "duration": "0.3"},
                        )
                        event = wait_for_event(id="translate_base")

                        node.send_output("look", pa.array([x, 0.3, z]))
                        trajectory = np.array(
                            [
                                [x, 0.3, z + 0.1, 0, 0, 0, 100],
                                [x, 0.3, z, 0, 0, 0, 0],
                                [x, 0.3, z + 0.1, 0, 0, 0, 0],
                                [x, 0.3, z + 0.1, -np.pi, 0, 0, 0],
                                [x, 0.3, z, -np.pi, 0, 0, 100],
                                [x, 0.3, z + 0.1, -np.pi, 0, 0, 100],
                                [x, 0.3, z + 0.1, 0, 0, 0, 100],
                            ],
                        ).ravel()
                        node.send_output("look", pa.array([x, 0.3, z]))

                        node.send_output(
                            "action_l_arm",
                            pa.array(trajectory),
                            metadata={"encoding": "xyzrpy", "duration": "0.8"},
                        )

                        event = wait_for_event(id="response_l_arm")

                        node.send_output(
                            "translate_base",
                            pa.array([0.0, -(y - 0.3), 0, 0, 0, 0]),
                            metadata={"encoding": "xyzrpy", "duration": "0.3"},
                        )
                        event = wait_for_event(id="translate_base")
                        node.send_output("look", pa.array([0.4, 0.0, 0.0]))

                        arm_holding_object = "left"
                        node.send_output("success", pa.array([True]))

                case "release":
                    if len(values) == 0:
                        continue
                    x = x + 0.05
                    z = z + 0.13

                    ## Clip the Maximum and minim values for the height of the arm to avoid collision or weird movement.
                    node.send_output("look", pa.array([x, y, z]))
                    if abs(y) > 0.5:
                        continue
                    if arm_holding_object is None:
                        print("No arm holding object!!!")
                        continue
                    elif arm_holding_object == "right":
                        # Send a mobile base command to move slightly left to facilitate the grasp
                        node.send_output(
                            "translate_base",
                            pa.array([0.0, y + 0.3, 0, 0, 0, 0]),
                            metadata={"encoding": "xyzrpy", "duration": "0.3"},
                        )
                        event = wait_for_event(id="translate_base", timeout=10)
                        node.send_output("look", pa.array([x, -0.3, z]))
                        trajectory = np.array(
                            [
                                [x, -0.3, z + 0.1, 0, 0, 0, 0],
                                [x, -0.3, z, 0, 0, 0, 100],
                                [x, -0.3, z + 0.1, 0, 0, 0, 100],
                            ],
                        ).ravel()

                        node.send_output(
                            "action_r_arm",
                            pa.array(trajectory),
                            metadata={"encoding": "xyzrpy", "duration": "0.3"},
                        )
                        event = wait_for_event(id="response_r_arm", timeout=10)
                        if not (
                            event is not None
                            and event[0] is not None
                            and event[0][0].as_py()
                        ):
                            trajectory = np.array(
                                [
                                    [x, 0.3, -0.2, 0, 0, 0, 0],
                                    [x, 0.3, -0.2, 0, 0, 0, 100],
                                    [x, 0.3, -0.2, 0, 0, 0, 100],
                                ],
                            ).ravel()

                            node.send_output(
                                "action_l_arm",
                                pa.array(trajectory),
                                metadata={"encoding": "xyzrpy", "duration": "0.3"},
                            )
                        node.send_output(
                            "action_r_arm",
                            pa.array(r_init_pose),
                            metadata={"encoding": "jointstate", "duration": "0.7"},
                        )

                        event = wait_for_event(id="response_r_arm", timeout=10)
                        node.send_output(
                            "translate_base",
                            pa.array([0.0, -(y + 0.3), 0, 0, 0, 0]),
                            metadata={"encoding": "xyzrpy", "duration": "0.3"},
                        )
                        event = wait_for_event(id="translate_base", timeout=10)
                        node.send_output("success", pa.array([True]))
                        node.send_output("look", pa.array([0.4, 0.0, 0.0]))

                    else:
                        node.send_output(
                            "translate_base",
                            pa.array([0.0, y - 0.3, 0, 0, 0, 0]),
                            metadata={"encoding": "xyzrpy", "duration": "0.3"},
                        )
                        event = wait_for_event(id="translate_base")
                        trajectory = np.array(
                            [
                                [x, 0.3, z + 0.1, 0, 0, 0, 0],
                                [x, 0.3, z, 0, 0, 0, 100],
                                [x, 0.3, z + 0.1, 0, 0, 0, 100],
                            ],
                        ).ravel()

                        node.send_output("look", pa.array([x, 0.3, z]))
                        node.send_output(
                            "action_l_arm",
                            pa.array(trajectory),
                            metadata={"encoding": "xyzrpy", "duration": "0.3"},
                        )
                        event = wait_for_event(id="response_l_arm")
                        if not (
                            event is not None
                            and event[0] is not None
                            and event[0][0].as_py()
                        ):
                            trajectory = np.array(
                                [
                                    [x, 0.3, -0.2, 0, 0, 0, 0],
                                    [x, 0.3, -0.2, 0, 0, 0, 100],
                                    [x, 0.3, -0.2, 0, 0, 0, 100],
                                ],
                            ).ravel()

                            node.send_output(
                                "action_l_arm",
                                pa.array(trajectory),
                                metadata={"encoding": "xyzrpy", "duration": "0.3"},
                            )

                        node.send_output(
                            "action_l_arm",
                            pa.array(l_init_pose),
                            metadata={"encoding": "jointstate", "duration": "0.7"},
                        )
                        event = wait_for_event(id="response_l_arm")

                        node.send_output(
                            "translate_base",
                            pa.array([0.0, -(y - 0.3), 0, 0, 0, 0]),
                            metadata={"encoding": "xyzrpy", "duration": "0.3"},
                        )
                        event = wait_for_event(id="translate_base")
                        node.send_output("success", pa.array([True]))
                        node.send_output("look", pa.array([0.4, 0.0, 0.0]))

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

        elif event["id"] == "follow_pose":
            node.send_output(
                "action_l_arm",
                pa.array(
                    [0, -15, 0, 0, 0, 0, 0, 0],
                ),
                metadata={"encoding": "jointstate", "duration": "0.75"},
            )

            event, cache = wait_for_event(id="response_l_arm", cache=cache)

            node.send_output(
                "action_r_arm",
                pa.array(
                    [0, 15, 0, 0, 0, 0, 0, 0],
                ),
                metadata={"encoding": "jointstate", "duration": "0.75"},
            )
            event, cache = wait_for_event(id="response_r_arm", cache=cache)

        elif event["id"] == "raise_arm_pose":
            node.send_output(
                "action_r_arm",
                pa.array(r_init_pose),
                metadata={"encoding": "jointstate", "duration": 2},
            )
            event = wait_for_event(id="response_r_arm", timeout=10)

            node.send_output(
                "action_l_arm",
                pa.array(l_init_pose),
                metadata={"encoding": "jointstate", "duration": 2},
            )
            event = wait_for_event(id="response_l_arm", timeout=10)

        elif event["id"] == "look_ahead":
            node.send_output("look", pa.array([0.7, 0, 0]))
