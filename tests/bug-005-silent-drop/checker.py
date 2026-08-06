#!/usr/bin/env python3
"""等待 data_in，收到则打印 PASS，10 秒超时则打印 BUG CONFIRMED"""
import time
from dora import Node

node = Node()
deadline = time.time() + 10
received = False

for event in node:
    if event["type"] == "INPUT" and event["id"] == "data_in":
        val = event["value"].to_pylist()[0]
        print(f"PASS: checker received data_in={val}")
        received = True
        break
    elif event["type"] == "STOP":
        break
    if time.time() > deadline:
        break

if not received:
    print("BUG CONFIRMED: checker did NOT receive data_in within 10s")
    print("➜ node-level `outputs` was silently dropped during resolution")
    print("  (libraries/core/src/descriptor/mod.rs:382)")
else:
    print("BUG NOT REPRODUCED — outputs wiring is working correctly")
