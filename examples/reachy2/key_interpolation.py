from dora import Node
import pyarrow as pa

node = Node()


for event in node:
    if event["type"] == "INPUT":
        if event["id"] == "keyboard":
            char = event["value"][0].as_py()[0]
            if char == "w":
                node.send_output("text", pa.array(["forward"]))
            elif char == "s":
                node.send_output("text", pa.array(["back"]))
            elif char == "d":
                node.send_output("text", pa.array(["right"]))
            elif char == "a":
                node.send_output("text", pa.array(["left"]))
            elif char == "q":
                node.send_output("text", pa.array(["grab"]))
            elif char == "e":
                node.send_output("text", pa.array(["release"]))
            elif char == "r":
                node.send_output("text", pa.array(["handwave"]))
            elif char == "f":
                node.send_output("text", pa.array(["fistbump"]))
            elif char == "g":
                node.send_output("text", pa.array(["handshake"]))
            elif char == "p":
                node.send_output("text", pa.array(["wait"]))
