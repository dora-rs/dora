"""Send data node for async dataflow example."""

import signal
import sys
import time

import numpy as np
import pyarrow as pa
from dora import Node


def handle_sigterm(signum, frame):
    """Handle SIGTERM for graceful shutdown."""
    print("Received SIGTERM, shutting down gracefully...")
    sys.exit(0)


# Register signal handler
signal.signal(signal.SIGTERM, handle_sigterm)

node = Node()

i = 0
for event in node:
    if i == 100:
        break
    else:
        i += 1
    now = time.perf_counter_ns()
    node.send_output("data", pa.array([np.uint64(now)]))
