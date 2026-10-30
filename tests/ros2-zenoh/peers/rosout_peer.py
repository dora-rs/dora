#!/usr/bin/env python3
# Checks what `ros2 node info` shows for a dora node created with
# `rosout=True` and no parameters: a `/rosout` publisher with rcl's QoS, and
# no parameter services.
import json
import time

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

NAME, NAMESPACE = "dora_native_peer", "/dora_test"

rclpy.init()
# rclpy gives this node its own `/rosout` publisher to compare with.
n = rclpy.create_node("dora_zenoh_rosout_peer")
print(json.dumps({"event": "READY"}), flush=True)


def check(ok, what):
    if not ok:
        raise SystemExit(what)


def rosout_publishers():
    return {
        (info.node_namespace, info.node_name): info
        for info in n.get_publishers_info_by_topic("/rosout")
    }


end = time.monotonic() + 15
while time.monotonic() < end and (NAMESPACE, NAME) not in rosout_publishers():
    rclpy.spin_once(n, timeout_sec=0.2)
check((NAMESPACE, NAME) in rosout_publishers(), "no /rosout publisher")
# Give anything the node declares after `/rosout` time to show up before
# checking that it serves nothing.
end = time.monotonic() + 2
while time.monotonic() < end:
    rclpy.spin_once(n, timeout_sec=0.2)
publishers = rosout_publishers()

topics = n.get_publisher_names_and_types_by_node(NAME, NAMESPACE)
check(("/rosout", ["rcl_interfaces/msg/Log"]) in topics, f"node info: {topics!r}")
services = n.get_service_names_and_types_by_node(NAME, NAMESPACE)
check(services == [], f"a node without parameters serves {services!r}")

dora = publishers[(NAMESPACE, NAME)]
qos = dora.qos_profile
check(qos.reliability == ReliabilityPolicy.RELIABLE, f"reliability: {qos}")
check(qos.durability == DurabilityPolicy.TRANSIENT_LOCAL, f"durability: {qos}")
check(qos.history == HistoryPolicy.KEEP_LAST, f"history: {qos}")
check(qos.depth == 1000, f"depth: {qos}")
# Humble's rclpy has no type hashes.
ros = publishers[(n.get_namespace(), n.get_name())]
dora_hash = str(getattr(dora, "topic_type_hash", None))
ros_hash = str(getattr(ros, "topic_type_hash", None))
check(dora_hash == ros_hash, f"type hash: {dora_hash} != {ros_hash}")
print(json.dumps({"event": "PASS"}), flush=True)
n.destroy_node()
rclpy.shutdown()
