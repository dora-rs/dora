from dora import Node
import pyarrow as pa

node = Node()

event = node.next()
if event["type"] == "INPUT":
    print(
        f"""Node received:
    id: {event["id"]},
    value: {event["value"]},
    metadata: {event["metadata"]}"""
    )
    node.send_output("speech", pa.array(["Hello World"]))
