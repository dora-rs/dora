#!/usr/bin/env python3
import argparse
import json
import rclpy
from example_interfaces.srv import AddTwoInts

p = argparse.ArgumentParser()
p.add_argument("role", choices=["client", "server"])
a = p.parse_args()
rclpy.init()
n = rclpy.create_node("dora_zenoh_service_peer")
if a.role == "server":
    state = {"done": False}
    def add(request, response):
        response.sum = request.a + request.b
        print(json.dumps({"event": "PASS", "sum": response.sum}), flush=True)
        state["done"] = True
        return response
    n.create_service(AddTwoInts, "/add_two_ints", add)
    print(json.dumps({"event": "READY", "role": a.role}), flush=True)
    end = n.get_clock().now().nanoseconds + 30_000_000_000
    while not state["done"] and n.get_clock().now().nanoseconds < end:
        rclpy.spin_once(n, timeout_sec=0.2)
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
