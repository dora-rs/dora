from dora import Node
import pyarrow as pa

node = Node("talker")

text = input("Enter a message: ")
data = pa.array([text])
node.send_output("message", data)
