#!/usr/bin/env python3
import argparse
import json
import rclpy
from rclpy.action import ActionClient, ActionServer
from example_interfaces.action import Fibonacci

p = argparse.ArgumentParser()
p.add_argument("role", choices=["client", "server"])
# Client only: send a plain goal and require SUCCEEDED instead of canceling it,
# for servers that do not handle cancel (the YAML bridge on DDS, see #4205).
p.add_argument("--no-cancel", action="store_true")
# Client only: the goal's Fibonacci order.
p.add_argument("--order", type=int, default=8)
# Client only: require the server to reject the goal (a YAML bridge with
# a `goal_response` input whose handler refuses it).
p.add_argument("--expect-reject", action="store_true")
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
    sent = client.send_goal_async(Fibonacci.Goal(order=a.order))
    rclpy.spin_until_future_complete(n, sent, timeout_sec=20)
    goal = sent.result()
    if a.expect_reject:
        if goal is None:
            raise SystemExit("goal response timed out")
        if goal.accepted:
            raise SystemExit("goal accepted, expected a rejection")
        print(json.dumps({"event": "PASS", "role": a.role}), flush=True)
        n.destroy_node()
        rclpy.shutdown()
        raise SystemExit(0)
    if goal is None or not goal.accepted:
        raise SystemExit("goal rejected or timed out")
    if not a.no_cancel:
        canceled = goal.cancel_goal_async()
        rclpy.spin_until_future_complete(n, canceled, timeout_sec=20)
        cancel_response = canceled.result()
        if cancel_response is None or not cancel_response.goals_canceling:
            raise SystemExit("cancel rejected or timed out")
    result = goal.get_result_async()
    rclpy.spin_until_future_complete(n, result, timeout_sec=30)
    value = result.result()
    if a.no_cancel:
        # Status 4 is SUCCEEDED. The sequence is only checked for being
        # non-empty: servers differ in the Fibonacci convention they use.
        if value is None or value.status != 4 or not value.result.sequence:
            raise SystemExit(f"plain action result mismatch or timeout: {value!r}")
    elif value is None or value.status != 5:
        raise SystemExit("canceled action result mismatch or timeout")
    print(json.dumps({"event": "PASS", "role": a.role}), flush=True)
n.destroy_node()
rclpy.shutdown()
