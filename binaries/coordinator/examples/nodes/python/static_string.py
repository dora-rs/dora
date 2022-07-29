import time

from dora import PyDoraNode

node = PyDoraNode()

for i in range(100):
    node.send_output("string", b"Hello World")
    time.sleep(0.1)

print("static string finished")
