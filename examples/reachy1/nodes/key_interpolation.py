"""TODO: Add docstring."""

import pyarrow as pa
from dora import Node

node = Node()


for event in node:
    if event["type"] == "INPUT":
        if event["id"] == "keyboard":
            char = event["value"][0].as_py()
            step = 0.1
            if char == "w":
                node.send_output("text", pa.array(["arm forward"]))
            elif char == "s":
                node.send_output("text", pa.array(["arm backward"]))
            elif char == "d":
                node.send_output("text", pa.array(["arm right"]))
            elif char == "a":
                node.send_output("text", pa.array(["arm left"]))
            elif char == "e":
                node.send_output("text", pa.array(["arm up"]))
            elif char == "q":
                node.send_output("text", pa.array(["arm down"]))
            elif char == "t":
                node.send_output("text", pa.array(["gripper close"]))
            elif char == "r":
                node.send_output("text", pa.array(["gripper open"]))
            elif char == "6":
                node.send_output("text", pa.array(["look right"]))
            elif char == "4":
                node.send_output("text", pa.array(["look left"]))
            elif char == "8":
                node.send_output("text", pa.array(["look up"]))
            elif char == "2":
                node.send_output("text", pa.array(["look down"]))
            elif char == "7":
                node.send_output("text", pa.array(["cry"]))
            elif char == "9":
                node.send_output("text", pa.array(["smile"]))
            elif char == "5":
                node.send_output("text", pa.array(["wait"]))
