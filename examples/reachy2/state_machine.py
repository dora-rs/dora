## State Machine
import time

import numpy as np
import pyarrow as pa
from dora import Node

node = Node()

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


r_default_pose = [
    38.058172640242475,
    0.07798708660299236,
    2.0084781702579564,
    -129.76629958820868,
    4.428130313456095,
    -9.272674208719419,
    354.280491569214,
    100,
]


def wait_for_event(id):
    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == id:
                return event["value"]


while True:

    ### === IDLE ===

    node.send_output(
        "action_arm",
        pa.array(r_default_pose),
        metadata={"encoding": "jointstate"},
    )
    wait_for_event(id="response_arm")
    node.send_output(
        "text_vlm",
        pa.array(["output the bounding box for human"]),
        metadata={"select_image": "image_left"},
    )

    while True:
        text = wait_for_event(id="text")

        if text is None:
            continue
        text = text[0].as_py()

        if "g" not in text:
            print("not g")
            continue
        else:
            break

    ### === TURNING ===

    # Trigger action once text from whisper is received
    # Move left. Overwrite this with movement if needed.
    node.send_output("action_base", pa.array([0.0, 0.0, 0.0, 0.0, 0.0, 1.57]))
    # Look straight
    node.send_output("look", pa.array([0.3, 0, 0.0]))
    # You can add additional actions here
    # ...

    event = wait_for_event(id="response_base")[0].as_py()
    if not event:
        # return to IDLE
        node.send_output("action_base", pa.array([0.0, 0.0, 0.0, 0.0, 0.0, -1.57]))
        event = wait_for_event(id="response_base")
        if event:
            continue
        else:
            break

    ### === GRABBING ===

    # Trigger action once base is done moving
    node.send_output(
        "text_vlm",
        pa.array(["output the bounding box for orange"]),
        metadata={"select_image": "image_depth"},
    )

    # Try pose and until one is successful
    while True:
        values = wait_for_event(id="pose")
        if values is None:
            continue
        values = values.to_numpy()
        x = values[0]
        y = values[1]
        z = values[2]
        x = x + 0.04
        z = np.clip(z + 0.01, -0.32, -0.22)
        if y > 0:
            y = y + 0.03

        node.send_output("look", pa.array([x, y, z]))

        # First pose is top
        # second one is lower
        # third one is top

        trajectory = np.array(
            [
                [x, y, -0.16, 0, 0, 0, 100],
                [x, y, z, 0, 0, 0, 0],
                [x, y, -0.16, 0, 0, 0, 0],
            ]
        ).ravel()

        node.send_output(
            "action_arm", pa.array(trajectory), metadata={"encoding": "xyzrpy"}
        )
        event = wait_for_event(id="response_arm")[0].as_py()
        if event:
            print("Success")
            break
        else:
            print("Failed")
            node.send_output(
                "action_arm",
                pa.array(r_default_pose),
                metadata={"encoding": "jointstate"},
            )
            event = wait_for_event(id="response_arm")
        time.sleep(0.3)

    ### === RELEASING ===

    # Trigger action once arm is done moving
    node.send_output("action_base", pa.array([0.0, 0.0, 0.0, 0.0, 0.0, -1.57]))

    if not event:
        print("Failed to move right")

    # Trigger action to release object
    node.send_output(
        "action_arm",
        pa.array(r_release_closed_pose),
        metadata={"encoding": "jointstate"},
    )
    event = wait_for_event(id="response_arm")
    node.send_output(
        "action_arm",
        pa.array(r_release_opened_pose),
        metadata={"encoding": "jointstate"},
    )
    event = wait_for_event(id="response_arm")

    if not event:
        print("Failed to release object")
