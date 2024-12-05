from dora import Node
import time
import pyarrow as pa
import numpy as np

node = Node()

i = 0
for event in node:
    if i == 1000:
        break
    else:
        i += 1
    now = time.clock_gettime_ns(0)
    node.send_output("data", pa.array([np.uint64(now)]))
