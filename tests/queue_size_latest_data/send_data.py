from dora import Node
import time
import pyarrow as pa

node = Node()

for i in range(10):
    now = time.time()
    node.send_output("data", pa.array([now]))
    time.sleep(1)
