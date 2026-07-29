#!/usr/bin/env python3
import argparse
import json
import rclpy
from rclpy.action import ActionClient, ActionServer
from example_interfaces.action import Fibonacci

p = argparse.ArgumentParser()
p.add_argument("role", choices=["client", "server"])
a = p.parse_args()
rclpy.init()
n = rclpy.create_node("dora_zenoh_action_peer")
if a.role == "server":
    state = {"done": False}
    def execute(goal):
        seq = [0, 1]
        for _ in range(2, goal.request.order):
            seq.append(seq[-1] + seq[-2])
            goal.publish_feedback(Fibonacci.Feedback(sequence=seq))
        goal.succeed()
        print(json.dumps({"event": "PASS", "role": a.role}), flush=True)
        state["done"] = True
        return Fibonacci.Result(sequence=seq)
    server = ActionServer(n, Fibonacci, "/fibonacci", execute)
    print(json.dumps({"event": "READY", "role": a.role}), flush=True)
    end = n.get_clock().now().nanoseconds + 40_000_000_000
    while not state["done"] and n.get_clock().now().nanoseconds < end:
        rclpy.spin_once(n, timeout_sec=0.2)
    if state["done"]:
        grace = n.get_clock().now().nanoseconds + 5_000_000_000
        while n.get_clock().now().nanoseconds < grace:
            rclpy.spin_once(n, timeout_sec=0.1)
    server.destroy()
else:
    client = ActionClient(n, Fibonacci, "/fibonacci")
    print(json.dumps({"event": "READY", "role": a.role}), flush=True)
    if not client.wait_for_server(timeout_sec=20):
        raise SystemExit("action discovery timeout")
    sent = client.send_goal_async(Fibonacci.Goal(order=8))
    rclpy.spin_until_future_complete(n, sent, timeout_sec=20)
    goal = sent.result()
    if goal is None or not goal.accepted:
        raise SystemExit("goal rejected or timed out")
    canceled = goal.cancel_goal_async()
    rclpy.spin_until_future_complete(n, canceled, timeout_sec=20)
    cancel_response = canceled.result()
    if cancel_response is None or not cancel_response.goals_canceling:
        raise SystemExit("cancel rejected or timed out")
    result = goal.get_result_async()
    rclpy.spin_until_future_complete(n, result, timeout_sec=30)
    value = result.result()
    if value is None or value.status != 5:
        raise SystemExit("canceled action result mismatch or timeout")
    print(json.dumps({"event": "PASS", "role": a.role}), flush=True)
n.destroy_node()
rclpy.shutdown()
