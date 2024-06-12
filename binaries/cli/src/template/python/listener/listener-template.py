from dora import Node
import pyarrow as pa

node = Node()

event = node.next()
if event["type"] == "INPUT":
    message = event["value"][0].as_py()
    print(
        f"""I heard {message} from {event["id"]}"""
    )