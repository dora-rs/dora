#!/usr/bin/env python3
"""发送 tick 信号，每 500ms 一次，共发 5 次后退出"""
import pyarrow as pa
from dora import Node
import time

node = Node()
for i in range(5):
    node.send_output("tick", pa.array([i], type=pa.int32()))
    time.sleep(0.5)
