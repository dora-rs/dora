from dora import Node
import pyarrow as pa

node = Node()


for event in node:
    if event["type"] == "INPUT":
        if event["id"] == "text":
            text = event["value"][0].as_py()
            text = text.lower()
            if "get" in text:
                node.send_output(
                    "text",
                    pa.array(
                        [
                            "Respond with left, right, forward, back, up, down or go home in order for the robotic arm to "
                            + text
                        ]
                    ),
                )
