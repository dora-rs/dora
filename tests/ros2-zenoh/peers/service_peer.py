#!/usr/bin/env python3
import argparse
import json
import os
import threading
import time
import rclpy
from example_interfaces.srv import AddTwoInts
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

p = argparse.ArgumentParser()
p.add_argument("role", choices=["client", "server"])
# Server only: answer each request after this many seconds.
p.add_argument("--delay", type=float, default=0.0)
# Server only: pass only once this many requests were being handled at the
# same time, i.e. the client had that many in flight (#4258).
p.add_argument("--expect-concurrent", type=int, default=1)
# Server only: handle one request at a time, like an rclpy server on the
# default executor, so later requests queue in the server (#4260).
p.add_argument("--one-at-a-time", action="store_true")
# Server only: pass only once this many requests were answered.
p.add_argument("--expect-requests", type=int, default=1)
# Server only: never answer the requests whose `a` is in this range, e.g. 4-6,
# as a server that drops them would (#4260).
p.add_argument("--drop", default="")
a = p.parse_args()
dropped = range(0)
if a.drop:
    first, last = (int(x) for x in a.drop.split("-"))
    dropped = range(first, last + 1)
rclpy.init()
n = rclpy.create_node("dora_zenoh_service_peer")
if a.role == "server":
    lock = threading.Lock()
    state = {"done": False, "active": 0, "peak": 0, "served": 0}
    def add(request, response):
        if request.a in dropped:
            # Never answered: the peer exits with these still held.
            while True:
                time.sleep(1)
        with lock:
            state["active"] += 1
            state["peak"] = max(state["peak"], state["active"])
        time.sleep(a.delay)
        response.sum = request.a + request.b
        with lock:
            state["active"] -= 1
            state["served"] += 1
            if (not state["done"] and state["peak"] >= a.expect_concurrent
                    and state["served"] >= a.expect_requests):
                print(json.dumps({"event": "PASS", "sum": response.sum, "concurrent": state["peak"],
                                  "served": state["served"]}), flush=True)
                state["done"] = True
        return response
    # Callbacks run in parallel, so a client with several requests in flight
    # gets them handled at the same time; or one at a time, as rclpy's default.
    group = None if a.one_at_a_time else ReentrantCallbackGroup()
    n.create_service(AddTwoInts, "/add_two_ints", add, callback_group=group)
    # Threads for the held requests too, so they do not block the others.
    executor = MultiThreadedExecutor(num_threads=4 + len(dropped))
    executor.add_node(n)
    print(json.dumps({"event": "READY", "role": a.role}), flush=True)
    end = n.get_clock().now().nanoseconds + int((30 + a.delay * a.expect_requests) * 1e9)
    while not (state["done"] and state["active"] == 0) and n.get_clock().now().nanoseconds < end:
        executor.spin_once(timeout_sec=0.2)
    if dropped:
        # The held callbacks never return, so neither would a clean shutdown.
        os._exit(0)
    executor.shutdown()
else:
    client = n.create_client(AddTwoInts, "/add_two_ints")
    print(json.dumps({"event": "READY", "role": a.role}), flush=True)
    if not client.wait_for_service(timeout_sec=20):
        raise SystemExit("service discovery timeout")
    future = client.call_async(AddTwoInts.Request(a=20, b=22))
    rclpy.spin_until_future_complete(n, future, timeout_sec=20)
    result = future.result()
    if result is None or result.sum != 42:
        raise SystemExit("response mismatch or timeout")
    print(json.dumps({"event": "PASS", "sum": result.sum}), flush=True)
n.destroy_node()
rclpy.shutdown()
