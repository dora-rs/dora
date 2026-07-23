#!/usr/bin/env python3
import argparse
import json
import time
import rclpy

p = argparse.ArgumentParser()
p.add_argument("mode", choices=["graph", "domain"])
a = p.parse_args()
rclpy.init()
n = rclpy.create_node("dora_zenoh_graph_probe")
print(json.dumps({"event": "READY", "mode": a.mode}), flush=True)
end = time.monotonic() + 15
seen = False
while time.monotonic() < end:
    rclpy.spin_once(n, timeout_sec=0.2)
    nodes = set(n.get_node_names_and_namespaces())
    topics = dict(n.get_topic_names_and_types())
    seen = ("dora_native_peer", "/dora_test") in nodes and "/graph_probe" in topics
    if a.mode == "graph" and seen:
        break
if a.mode == "graph" and not seen:
    raise SystemExit("Dora graph entity not visible")
if a.mode == "domain" and seen:
    raise SystemExit("domain 42 leaked into domain 0")
print(json.dumps({"event": "PASS", "mode": a.mode}), flush=True)
n.destroy_node()
rclpy.shutdown()
