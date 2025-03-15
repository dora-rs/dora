from dora import Node
import pyarrow as pa

node = Node()


for event in node:
    if event["type"] == "INPUT":
        if event["id"] == "text":
            text = event["value"][0].as_py()
            text = text.lower()
            text = text.replace("saving", "save")
            text = text.replace("teaching", "teach")
            text = text.replace("turning left", "yaw left")
            text = text.replace("turning right", "yaw right")
            text = text.replace("turning up", "pitch up")
            text = text.replace("turning down", "pitch down")
            text = text.replace("rolling right", "roll right")
            text = text.replace("rolling left", "roll left")
            text = text.replace("turning shoulder right", "yaw shoulder right")
            text = text.replace("turning shoulder left", "yaw shoulder left")
            node.send_output("text", pa.array([text]))
