import pyarrow as pa
from dora import Node

node = Node()


for event in node:
    if event["type"] == "INPUT":
        text = event["value"][0].as_py()
        text = text.lower()
        text = text.replace(".", "")
        head_step = 5
        step = 0.02

        if text == "look right":
            node.send_output("head_action", pa.array([0, -head_step, 0]))
        elif text == "look left":
            node.send_output("head_action", pa.array([0, head_step, 0]))
        elif text == "look up":
            node.send_output(
                "head_action", pa.array([head_step / 2, 0, -head_step / 2]),
            )
        elif text == "look down":
            node.send_output(
                "head_action", pa.array([-head_step / 2, 0, head_step / 2]),
            )
        elif text == "look up":
            node.send_output(
                "head_action", pa.array([head_step / 2, 0, -head_step / 2]),
            )
        elif text == "look down":
            node.send_output(
                "head_action", pa.array([-head_step / 2, 0, head_step / 2]),
            )
        elif text == "smile":
            node.send_output("antenna_action", pa.array(["smile"]))
        elif text == "cry":
            node.send_output("antenna_action", pa.array(["cry"]))
        elif text == "forward":
            node.send_output("r_arm_action", pa.array([step, 0, 0]))
        elif text == "backward":
            node.send_output("r_arm_action", pa.array([-step, 0, 0]))
        elif text == "right":
            node.send_output("r_arm_action", pa.array([0, -step, 0]))
        elif text == "left":
            node.send_output("r_arm_action", pa.array([0, step, 0]))
        elif text == "up":
            node.send_output("r_arm_action", pa.array([0, 0, step]))
        elif text == "down":
            node.send_output("r_arm_action", pa.array([0, 0, -step]))
        elif text == "open":
            node.send_output(
                "question",
                pa.array(
                    [
                        "Respond with right, left, forward, backward, open, or close to grab the trash",
                    ],
                ),
            )

            node.send_output("gripper_action", pa.array([-100]))
        elif text == "close":
            node.send_output(
                "question",
                pa.array(
                    [
                        "Respond with right, left, forward, backward, open, or close to put the trash in your hand in the right bin",
                    ],
                ),
            )
            node.send_output("gripper_action", pa.array([100]))
