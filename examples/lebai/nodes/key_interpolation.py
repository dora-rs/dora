from dora import Node
import pyarrow as pa

node = Node()


for event in node:
    if event["type"] == "INPUT":
        if event["id"] == "keyboard":
            char = event["value"][0].as_py()

            if char == "w":
                node.send_output("text", pa.array(["forward"]))
            elif char == "s":
                node.send_output("text", pa.array(["back"]))
            elif char == "c":
                node.send_output("text", pa.array([" go home"]))
            elif char == "d":
                node.send_output("text", pa.array(["right"]))
            elif char == "a":
                node.send_output("text", pa.array(["left"]))
            elif char == "e":
                node.send_output("text", pa.array(["up"]))
            elif char == "q":
                node.send_output("text", pa.array(["down"]))
            elif char == "t":
                node.send_output("text", pa.array(["close"]))
            elif char == "r":
                node.send_output("text", pa.array(["open"]))
            elif char == "6":
                node.send_output("text", pa.array(["yaw right"]))
            elif char == "4":
                node.send_output("text", pa.array(["yaw left"]))
            elif char == "3":
                node.send_output("text", pa.array(["yaw shoulder right"]))
            elif char == "1":
                node.send_output("text", pa.array(["yaw shoulder left"]))
            elif char == "8":
                node.send_output("text", pa.array(["pitch up"]))
            elif char == "2":
                node.send_output("text", pa.array(["pitch down"]))
            elif char == "7":
                node.send_output("text", pa.array(["roll left"]))
            elif char == "9":
                node.send_output("text", pa.array(["roll right"]))
            elif char == "x":
                node.send_output("text", pa.array(["stop"]))
            elif char == "j":
                node.send_output("text", pa.array([""]))
