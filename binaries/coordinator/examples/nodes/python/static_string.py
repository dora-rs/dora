import time

from dora import Node

node = Node()

for i in range(100):
    node.send_output("string", b"Hello World")
    time.sleep(0.1)

print("static string finished")
