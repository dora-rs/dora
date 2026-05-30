#!/usr/bin/env python
"""Python ROS2 service server example.

Serves the ROS2 `example_interfaces/AddTwoInts` service from a dora Python node
using the `create_service_server` API of the dora ROS2 bridge. Requests are
polled on each dora timer tick and answered with the sum.
"""

import pyarrow as pa
from dora import Node, Ros2Context, Ros2NodeOptions, Ros2QosPolicies

# Create a ROS2 context + node and advertise the service.
ros2_context = Ros2Context()
ros2_node = ros2_context.new_node(
    "service_server",
    "/ros2_demo",
    Ros2NodeOptions(rosout=True),
)
service_qos = Ros2QosPolicies(reliable=True, max_blocking_time=0.1)
add_two_ints = ros2_node.create_service_server(
    "/add_two_ints",
    "example_interfaces/AddTwoInts",
    service_qos,
)

# Poll for requests on each dora tick and answer them.
dora_node = Node()
handled = 0
for _ in range(200):
    event = dora_node.next()
    if event is None:
        break
    if event["type"] == "INPUT" and event["id"] == "tick":
        while True:
            req = add_two_ints.take_request(timeout_s=0.05)
            if req is None:
                break
            request_id, value = req
            args = value[0].as_py()
            total = args["a"] + args["b"]
            add_two_ints.send_response(request_id, pa.array([{"sum": total}]))
            handled += 1
            print(f"server: {args['a']} + {args['b']} = {total}", flush=True)
    if handled >= 3:
        break

assert handled > 0, "service server handled no requests"
print(f"PYTHON SERVICE SERVER OK ({handled} handled)", flush=True)
