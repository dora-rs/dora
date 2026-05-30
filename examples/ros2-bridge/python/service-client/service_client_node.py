#!/usr/bin/env python
"""Python ROS2 service client example.

Calls the ROS2 `example_interfaces/AddTwoInts` service (provided by the
`examples_rclcpp_minimal_service` node) from a dora Python node, using the
`create_service_client` API of the dora ROS2 bridge.
"""

import pyarrow as pa
from dora import Node, Ros2Context, Ros2NodeOptions, Ros2QosPolicies

# Create a ROS2 context + node.
ros2_context = Ros2Context()
ros2_node = ros2_context.new_node(
    "service_client",
    "/ros2_demo",
    Ros2NodeOptions(rosout=True),
)

# ROS2 services require reliable QoS.
service_qos = Ros2QosPolicies(reliable=True, max_blocking_time=0.1)

# Create the service client for /add_two_ints. This waits (bounded) for the
# service to become available before returning.
add_two_ints = ros2_node.create_service_client(
    "/add_two_ints",
    "example_interfaces/AddTwoInts",
    service_qos,
)

# Drive calls from a dora timer tick.
dora_node = Node()
responses_received = 0
for i in range(20):
    event = dora_node.next()
    if event is None:
        break
    if event["type"] == "INPUT" and event["id"] == "tick":
        response = add_two_ints.call(pa.array([{"a": i, "b": 1}]))
        result = response[0].as_py()
        print(f"tick {i}: {i} + 1 = {result['sum']}", flush=True)
        assert result["sum"] == i + 1, f"expected {i + 1}, got {result['sum']}"
        responses_received += 1
        if responses_received >= 3:
            break

assert responses_received > 0, "no service responses received"
print("PYTHON SERVICE CLIENT OK", flush=True)
