from dora import Node
import pyarrow as pa

node = Node("talker")

node.send_output("message", pa.array(["Hello", "World"]))
