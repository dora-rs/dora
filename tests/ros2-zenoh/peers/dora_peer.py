#!/usr/bin/env python3
import argparse
import json
import os
import time

import pyarrow as pa
import dora_ros2_bridge_python as dora

p = argparse.ArgumentParser()
p.add_argument("case")
p.add_argument("--profile", choices=["humble", "rep2016"], required=True)
p.add_argument("--ament", required=True)
p.add_argument("--config", required=True)
a = p.parse_args()

if a.case == "domain":
    os.environ["ROS_DOMAIN_ID"] = "42"

transport = dora.Ros2Transport.zenoh(a.profile, a.config)
context = dora.Ros2Context([a.ament], transport)
parameters = {"speed": 1.5, "gains": [1.0, 2.0]} if a.case == "parameters" else None
node = context.new_node(
    "dora_native_peer",
    "/dora_test",
    dora.Ros2NodeOptions(rosout=a.case == "rosout"),
    parameters,
)
qos = dora.Ros2QosPolicies(reliable=True, keep_last=10)

def emit(event, **fields):
    print(json.dumps({"event": event, "case": a.case, **fields}), flush=True)

def wait_until(read, timeout=20):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        value = read()
        if value is not None:
            return value
        time.sleep(0.02)
    raise TimeoutError(a.case)

emit("READY")
if a.case == "topic-pub":
    topic = node.create_topic("/dora_zenoh_test", "std_msgs/String", qos)
    publisher = node.create_publisher(topic)
    for _ in range(50):
        publisher.publish(pa.array([{"data": "dora-rmw-zenoh"}]))
        time.sleep(0.1)
elif a.case == "topic-sub":
    topic = node.create_topic("/dora_zenoh_test", "std_msgs/String", qos)
    subscription = node.create_subscription(topic)
    value = wait_until(subscription.next)
    if value[0].as_py()["data"] != "dora-rmw-zenoh":
        raise SystemExit("topic payload mismatch")
elif a.case == "service-client":
    client = node.create_service_client(
        "/add_two_ints", "example_interfaces/AddTwoInts", qos
    )
    response = client.call(pa.array([{"a": 20, "b": 22}]), timeout_s=20)
    if response[0].as_py()["sum"] != 42:
        raise SystemExit("service response mismatch")
elif a.case == "service-server":
    server = node.create_service_server(
        "/add_two_ints", "example_interfaces/AddTwoInts", qos
    )
    request_id, request = wait_until(lambda: server.take_request(timeout_s=0.1))
    values = request[0].as_py()
    server.send_response(request_id, pa.array([{"sum": values["a"] + values["b"]}]))
elif a.case == "action-client":
    client = node.create_action_client(
        "/fibonacci", "example_interfaces/Fibonacci", qos
    )
    goal_id = client.send_goal(
        pa.array([{"order": 8}], type=pa.struct([pa.field("order", pa.int32())])),
        timeout_s=20,
    )
    if goal_id is None:
        raise SystemExit("action goal rejected")
    result = client.take_result(goal_id, timeout_s=30)
    if result is None or result[1][0].as_py()["sequence"][-1] != 13:
        raise SystemExit(f"action result mismatch: {result!r}")
elif a.case == "action-server":
    server = node.create_action_server(
        "/fibonacci", "example_interfaces/Fibonacci", qos
    )
    goal_id, goal = wait_until(lambda: server.take_goal(timeout_s=0.1), 30)
    order = goal[0].as_py()["order"]
    sequence = [0, 1]
    canceled = wait_until(lambda: server.take_cancel(timeout_s=0.1), 20)
    if goal_id not in canceled:
        raise SystemExit(f"cancel did not identify active goal: {canceled!r}")
    result = pa.array(
        [{"sequence": sequence}],
        type=pa.struct([pa.field("sequence", pa.list_(pa.int32()))]),
    )
    server.send_result(goal_id, result, status="canceled", timeout_s=30)
elif a.case in ("graph", "domain"):
    topic = node.create_topic("/graph_probe", "std_msgs/String", qos)
    publisher = node.create_publisher(topic)
    end = time.monotonic() + 16
    while time.monotonic() < end:
        publisher.publish(pa.array([{"data": "graph"}]))
        time.sleep(0.2)
elif a.case == "namespace":
    topic = node.create_topic("/robot/chatter", "std_msgs/String", qos)
    publisher = node.create_publisher(topic)
    for _ in range(50):
        publisher.publish(pa.array([{"data": "dora-rmw-zenoh"}]))
        time.sleep(0.1)
elif a.case == "qos-transient-local":
    transient = dora.Ros2QosPolicies(
        durability=dora.Ros2Durability.TransientLocal,
        reliable=True,
        keep_last=1,
    )
    topic = node.create_topic("/dora_zenoh_test", "std_msgs/String", transient)
    subscription = node.create_subscription(topic)
    value = wait_until(subscription.next)
    if value[0].as_py()["data"] != "dora-rmw-zenoh":
        raise SystemExit("transient-local replay mismatch")
elif a.case == "parameters":
    # The ROS peer sets speed to 8.0 last; answer with a local set, then wait
    # for its "done" (10.0) and give that reply time to leave.
    wait_until(lambda: True if node.get_parameter("speed") == 8.0 else None, 40)
    node.set_parameter("speed", 9.0)
    wait_until(lambda: True if node.get_parameter("speed") == 10.0 else None)
    time.sleep(1)
elif a.case == "rosout":
    # Longer than the ROS peer waits for `/rosout` (15 s) plus its 2 s settle.
    time.sleep(20)
else:
    raise SystemExit(f"unsupported Dora peer case: {a.case}")
emit("PASS")
