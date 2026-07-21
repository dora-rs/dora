#!/usr/bin/env python3
import argparse
import json
import rclpy
from std_msgs.msg import String

p = argparse.ArgumentParser()
p.add_argument("role", choices=["publish", "publish-transient", "subscribe"])
p.add_argument("--topic", default="/dora_zenoh_test")
a = p.parse_args()
rclpy.init()
n = rclpy.create_node("dora_zenoh_topic_peer")
if a.role in ("publish", "publish-transient"):
    if a.role == "publish-transient":
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    else:
        qos = 10
    pub = n.create_publisher(String, a.topic, qos)
    print(json.dumps({"event": "READY", "role": a.role}), flush=True)
    end = n.get_clock().now().nanoseconds + 10_000_000_000
    if a.role == "publish-transient":
        pub.publish(String(data="dora-rmw-zenoh"))
    while n.get_clock().now().nanoseconds < end:
        if a.role == "publish":
            pub.publish(String(data="dora-rmw-zenoh"))
        rclpy.spin_once(n, timeout_sec=0.1 if a.role == "publish" else 1.0)
    print(json.dumps({"event": "PASS", "role": a.role}), flush=True)
else:
    state = {"ok": False}
    def receive(msg):
        state["ok"] = msg.data == "dora-rmw-zenoh"
    n.create_subscription(String, a.topic, receive, 10)
    print(json.dumps({"event": "READY", "role": a.role}), flush=True)
    end = n.get_clock().now().nanoseconds + 10_000_000_000
    while not state["ok"] and n.get_clock().now().nanoseconds < end:
        rclpy.spin_once(n, timeout_sec=0.2)
    if not state["ok"]:
        raise SystemExit("payload mismatch or timeout")
    print(json.dumps({"event": "PASS", "role": a.role}), flush=True)
n.destroy_node()
rclpy.shutdown()
