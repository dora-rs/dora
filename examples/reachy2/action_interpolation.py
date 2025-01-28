from dora import Node
import pyarrow as pa
import numpy as np

node = Node()


for event in node:
    if event["type"] == "INPUT":
        char = event["value"][0].as_py()
        if char == "left":
            node.send_output(
                "action",
                pa.array([0.0, 0.0, 0.0, 0.0, 0.0, np.deg2rad(10.0)]),
            )
        elif char == "right":
            node.send_output(
                "action",
                pa.array([0.0, 0.0, 0.0, 0.0, 0.0, np.deg2rad(-10.0)]),
            )
        elif char == "forward":
            node.send_output("action", pa.array([0.2, 0.0, 0.0, 0.0, 0.0, 0.0]))
        elif char == "back":
            node.send_output("action", pa.array([-0.2, 0.0, 0.0, 0.0, 0.0, 0.0]))
        elif char == "stop":
            node.send_output("action", pa.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        elif char == "handwave":
            node.send_output("action_arm", pa.array(["handwave"]))
        elif char == "grab":
            node.send_output("action_arm", pa.array(["grab"]))
        elif char == "release":
            node.send_output("action_arm", pa.array(["release"]))
        elif char == "fistbump":
            node.send_output("action_arm", pa.array(["fistbump"]))
        elif char == "handshake":
            node.send_output("action_arm", pa.array(["handshake"]))
        elif char == "wait":
            node.send_output("action_arm", pa.array(["wait"]))
