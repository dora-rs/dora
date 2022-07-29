from dora import PyDoraNode

node = PyDoraNode.init_from_env()
import time

for i in range(100):
    node.send_output("time", b"awef")
    time.sleep(0.1)

print("printer finished")
