"""Module for interpolating between keyframe positions.

This module provides functionality for creating smooth transitions between
keyframe positions in robot motion sequences.
"""

import pyarrow as pa
from dora import Node

node = Node()


for event in node:
    if event["type"] == "INPUT":
        if event["id"] == "keyboard":
            char = event["value"][0].as_py()

            if char == "w":
                node.send_output("text", pa.array(["forward"]), {"primitive": "text"})
            elif char == "s":
                node.send_output("text", pa.array(["back"]), {"primitive": "text"})
            elif char == "c":
                node.send_output("text", pa.array([" go home"]), {"primitive": "text"})
            elif char == "d":
                node.send_output("text", pa.array(["right"]), {"primitive": "text"})
            elif char == "a":
                node.send_output("text", pa.array(["left"]), {"primitive": "text"})
            elif char == "e":
                node.send_output("text", pa.array(["up"]), {"primitive": "text"})
            elif char == "q":
                node.send_output("text", pa.array(["down"]), {"primitive": "text"})
            elif char == "t":
                node.send_output("text", pa.array(["close"]), {"primitive": "text"})
            elif char == "r":
                node.send_output("text", pa.array(["open"]), {"primitive": "text"})
            elif char == "6":
                node.send_output("text", pa.array(["yaw right"]), {"primitive": "text"})
            elif char == "4":
                node.send_output("text", pa.array(["yaw left"]), {"primitive": "text"})
            elif char == "3":
                node.send_output("text", pa.array(["yaw shoulder right"]), {"primitive": "text"})
            elif char == "1":
                node.send_output("text", pa.array(["yaw shoulder left"]), {"primitive": "text"})
            elif char == "8":
                node.send_output("text", pa.array(["pitch up"]), {"primitive": "text"})
            elif char == "2":
                node.send_output("text", pa.array(["pitch down"]), {"primitive": "text"})
            elif char == "7":
                node.send_output("text", pa.array(["roll left"]), {"primitive": "text"})
            elif char == "9":
                node.send_output("text", pa.array(["roll right"]), {"primitive": "text"})
            elif char == "x":
                node.send_output("text", pa.array(["stop"]), {"primitive": "text"})
            elif char == "j":
                node.send_output("text", pa.array([""]), {"primitive": "text"})
