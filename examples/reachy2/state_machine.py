# State Machine
import os
import time

import numpy as np
import pyarrow as pa
from dora import Node

node = Node()

ACTIVATION_WORDS = os.getenv("ACTIVATION_WORDS", "").split()
TABLE_HEIGHT = float(os.getenv("TABLE_HEIGHT", "-0.33"))

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


def wait_for_event(id):
    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == id:
                return event["value"]


def wait_for_events(ids: list[str]):
    response = {}
    for event in node:
        if event["type"] == "INPUT":
            if event["id"] in ids:
                response[event["id"]] = event["value"]
                if len(response) == len(ids):
                    return event["value"]


while True:
    ### === IDLE ===

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
    wait_for_events(ids=["response_r_arm", "response_l_arm"])

    node.send_output(
        "text_vlm",
        pa.array(["output the bounding box for human"]),
        metadata={"image_id": "image_left"},
    )

    while True:
        text = wait_for_event(id="text")
        if text is None:
            continue
        text = text[0].as_py()
        print(text)

        words = text.lower().split()
        if len(ACTIVATION_WORDS) > 0 and all(
            word not in ACTIVATION_WORDS for word in words
        ):
            continue
        else:
            break

    ### === TURNING ===

    # Trigger action once text from whisper is received
    # Move left. Overwrite this with your desired movement..
    node.send_output("action_base", pa.array([0.0, 0.0, 0.0, 0.0, 0.0, 1.57]))
    # Look straight
    node.send_output("look", pa.array([0.3, 0, -0.1]))
    # You can add additional actions here
    # ...

    event = wait_for_event(id="response_base")[0].as_py()
    if not event:
        # return to IDLE
        node.send_output("action_base", pa.array([0.0, 0.0, 0.0, 0.0, 0.0, -1.57]))
        event = wait_for_event(id="response_base")[0].as_py()
        if event:
            continue
        else:
            break

    ### === GRABBING ===

    # Trigger action once base is done moving
    node.send_output(
        "text_vlm",
        pa.array([f"Given the prompt: {text}. Output bounding box for this action"]),
        metadata={"image_id": "image_depth"},
    )

    arm_holding_object = None

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

        ## Clip the Maximum and minim values for the height of the arm to avoid collision or weird movement.
        z = np.clip(z, TABLE_HEIGHT, -0.22)
        node.send_output("look", pa.array([x, y, z]))
        trajectory = np.array(
            [
                [x, y, -0.16, 0, 0, 0, 100],
                [x, y, z, 0, 0, 0, 0],
                [x, y, -0.16, 0, 0, 0, 0],
            ]
        ).ravel()

        if y < 0:
            node.send_output(
                "action_r_arm",
                pa.array(trajectory),
                metadata={"encoding": "xyzrpy", "duration": "0.75"},
            )
            event = wait_for_event(id="response_r_arm")[0].as_py()
            if event:
                print("Success")
                arm_holding_object = "right"
                break
            else:
                print("Failed: x: ", x, " y: ", y, " z: ", z)
                node.send_output(
                    "action_r_arm",
                    pa.array(r_init_pose),
                    metadata={"encoding": "jointstate", "duration": "1"},
                )
                event = wait_for_event(id="response_r_arm")
        else:
            node.send_output(
                "action_l_arm",
                pa.array(trajectory),
                metadata={"encoding": "xyzrpy", "duration": "0.75"},
            )
            event = wait_for_event(id="response_l_arm")[0].as_py()
            if event:
                print("Success")
                arm_holding_object = "left"
                break
            else:
                print("Failed")
                node.send_output(
                    "action_l_arm",
                    pa.array(l_init_pose),
                    metadata={"encoding": "jointstate", "duration": "1"},
                )
                event = wait_for_event(id="response_l_arm")

        time.sleep(0.3)

    ### === RELEASING ===

    # Trigger action once r_arm is done moving
    node.send_output("action_base", pa.array([0.0, 0.0, 0.0, 0.0, 0.0, -1.57]))
    event = wait_for_event(id="response_base")[0].as_py()

    if not event:
        print("Failed to move right")

    # Trigger action to release object
    if arm_holding_object == "right":
        node.send_output(
            "action_r_arm",
            pa.array(r_release_closed_pose),
            metadata={"encoding": "jointstate"},
        )
        event = wait_for_event(id="response_r_arm")
        node.send_output(
            "action_r_arm",
            pa.array(r_release_opened_pose),
            metadata={"encoding": "jointstate"},
        )
        event = wait_for_event(id="response_r_arm")
    else:
        node.send_output(
            "action_l_arm",
            pa.array(l_release_closed_pose),
            metadata={"encoding": "jointstate"},
        )
        event = wait_for_event(id="response_l_arm")
        node.send_output(
            "action_l_arm",
            pa.array(l_release_opened_pose),
            metadata={"encoding": "jointstate"},
        )
        event = wait_for_event(id="response_l_arm")

    if not event:
        print("Failed to release object")
