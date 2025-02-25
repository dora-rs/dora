# State Machine
import json
import os

import numpy as np
import pyarrow as pa
from dora import Node

IMAGE_RESIZE_RATIO = float(os.getenv("IMAGE_RESIZE_RATIO", "1.0"))
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


def extract_bboxes(json_text) -> (np.ndarray, np.ndarray):
    """
    Extracts bounding boxes from a JSON string with markdown markers and returns them as a NumPy array.

    Parameters:
    json_text (str): JSON string containing bounding box data, including ```json markers.

    Returns:
    np.ndarray: NumPy array of bounding boxes.
    """
    # Ensure all lines are stripped of whitespace and markers
    lines = json_text.strip().splitlines()

    # Filter out lines that are markdown markers
    clean_lines = [line for line in lines if not line.strip().startswith("```")]

    # Join the lines back into a single string
    clean_text = "\n".join(clean_lines)
    # Parse the cleaned JSON text
    try:
        data = json.loads(clean_text)

        # Extract bounding boxes
        bboxes = [item["bbox_2d"] for item in data]
        labels = [item["label"] for item in data]

        return np.array(bboxes), np.array(labels)
    except Exception as _e:  # noqa
        pass
    return None, None


def handle_speech(last_text):
    words = last_text.lower().split()
    if len(ACTIVATION_WORDS) > 0 and any(word in ACTIVATION_WORDS for word in words):

        node.send_output(
            "text_vlm",
            pa.array(
                [
                    f"Given the prompt: {cache['text']}. Output the two bounding boxes for the two objects"
                ]
            ),
            metadata={"image_id": "image_depth"},
        )
        node.send_output(
            "prompt",
            pa.array([cache["text"]]),
            metadata={"image_id": "image_depth"},
        )
        print("sending text")


def wait_for_event(id, timeout=None, cache={}):

    while True:
        event = node.next(timeout=timeout)
        if event is None:
            return None, cache
        if event["type"] == "INPUT":
            cache[event["id"]] = event["value"]
            if event["id"] == "text":
                cache[event["id"]] = event["value"][0].as_py()
                handle_speech(event["value"][0].as_py())
            elif event["id"] == id:
                return event["value"], cache

        elif event["type"] == "ERROR":
            return None, cache


def wait_for_events(ids: list[str], timeout=None, cache={}):
    response = {}
    while True:
        event = node.next(timeout=timeout)
        if event is None:
            return None, cache
        if event["type"] == "INPUT":
            cache[event["id"]] = event["value"]
            if event["id"] == "text":
                cache[event["id"]] = event["value"][0].as_py()
                handle_speech(event["value"][0].as_py())
            elif event["id"] in ids:
                response[event["id"]] = event["value"]
                if len(response) == len(ids):
                    return response, cache
        elif event["type"] == "ERROR":
            return None, cache


def get_prompt():
    text = wait_for_event(id="text", timeout=0.3)
    if text is None:
        return
    text = text[0].as_py()

    words = text.lower().split()
    if len(ACTIVATION_WORDS) > 0 and all(
        word not in ACTIVATION_WORDS for word in words
    ):
        return
    else:
        return text


last_text = ""
cache = {"text": "Put the yellow cube in the box"}

while True:
    ### === IDLE ===

    node.send_output(
        "action_r_arm",
        pa.array(r_init_pose),
        metadata={"encoding": "jointstate", "duration": 1},
    )
    node.send_output(
        "action_l_arm",
        pa.array(l_init_pose),
        metadata={"encoding": "jointstate", "duration": 1},
    )
    _, cache = wait_for_events(
        ids=["response_r_arm", "response_l_arm"], timeout=2, cache=cache
    )
    handle_speech(cache["text"])

    ### === TURNING ===

    # Trigger action once text from whisper is received
    # Move left. Overwrite this with your desired movement..
    # node.send_output("action_base", pa.array([0.0, 0.0, 0.0, 0.0, 0.0, 1.57]))
    # Look straight
    # node.send_output("look", pa.array([0.3, 0, -0.1]))
    # You can add additional actions here
    # ...

    # event = wait_for_event(id="response_base")[0].as_py()
    # if not event:
    ## return to IDLE
    # node.send_output("action_base", pa.array([0.0, 0.0, 0.0, 0.0, 0.0, -1.57]))
    # event = wait_for_event(id="response_base")[0].as_py()
    # if event:
    # continue
    # else:
    # break

    ### === GRABBING ===

    # Trigger action once base is done moving
    # node.send_output(
    # "text_vlm",
    # pa.array([f"Given the prompt: {text}. Output bounding box for this action"]),
    # metadata={"image_id": "image_depth"},
    # )
    arm_holding_object = None
    # Try pose and until one is successful
    text, cache = wait_for_event(id="text", timeout=0.3, cache=cache)
    while True:
        values, cache = wait_for_event(id="pose", cache=cache)

        if values is None:
            continue
        values = values.to_numpy().reshape((-1, 6))
        if len(values) < 2:
            continue
        x = values[0][0]
        y = values[0][1]
        z = values[0][2]
        dest_x = values[1][0]
        dest_y = values[1][1]
        dest_z = values[1][2]
        x = x + 0.01
        print("x: ", x, " y: ", y, " z: ", z)

        ## Clip the Maximum and minim values for the height of the arm to avoid collision or weird movement.
        z = np.max((z, TABLE_HEIGHT))
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
            event, cache = wait_for_event(id="response_r_arm", timeout=5, cache=cache)
            if event is not None and event[0].as_py():
                print("Success")
                arm_holding_object = "right"
                break
            else:
                print("Failed: x: ", x, " y: ", y, " z: ", z)
                node.send_output(
                    "action_r_arm",
                    pa.array(r_init_pose),
                    metadata={"encoding": "jointstate", "duration": "1.3"},
                )
                event, cache = wait_for_event(id="response_r_arm", cache=cache)
        else:
            y += 0.03
            node.send_output(
                "action_l_arm",
                pa.array(trajectory),
                metadata={"encoding": "xyzrpy", "duration": "0.75"},
            )
            event, cache = wait_for_event(id="response_l_arm", timeout=5, cache=cache)
            if event is not None and event[0].as_py():
                print("Success")
                arm_holding_object = "left"
                break
            else:
                print("Failed")
                node.send_output(
                    "action_l_arm",
                    pa.array(l_init_pose),
                    metadata={"encoding": "jointstate", "duration": "1.3"},
                )
                event, cache = wait_for_event(id="response_l_arm", cache=cache)
    ### === RELEASING ===

    # Trigger action once r_arm is done moving
    # node.send_output("action_base", pa.array([0.0, 0.0, 0.0, 0.0, 0.0, -1.57]))
    # event = wait_for_event(id="response_base")[0].as_py()

    # if not event:
    #    print("Failed to move right")

    # Trigger action to release object
    if arm_holding_object == "right":
        node.send_output(
            "action_r_arm",
            pa.array(
                [
                    dest_x,
                    dest_y,
                    dest_z + 0.15,
                    0,
                    0,
                    0,
                    100,
                ],
            ),
            metadata={"encoding": "xyzrpy", "duration": "0.75"},
        )
        event, cache = wait_for_event(id="response_r_arm", cache=cache)
    else:
        node.send_output(
            "action_l_arm",
            pa.array(
                [
                    dest_x,
                    dest_y,
                    dest_z + 0.15,
                    0,
                    0,
                    0,
                    100,
                ]
            ),
            metadata={"encoding": "xyzrpy", "duration": "0.75"},
        )
        event, cache = wait_for_event(id="response_l_arm", cache=cache)

    if not event:
        print("Failed to release object")
