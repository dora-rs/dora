from dora import Node
import time
import pyarrow as pa
import numpy as np

node = Node()

for i in range(10):
    now = time.time()
    node.send_output("data", pa.array([np.uint64(now)]))
    time.sleep(1)
