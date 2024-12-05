from dora import Node
import time
import pyarrow as pa
import numpy as np

node = Node()

for event in node:
    now = time.clock_gettime_ns(0)
    node.send_output("data", pa.array([np.uint64(now)]))
