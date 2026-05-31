#!/usr/bin/env python
"""Python ROS2 action server example.

Serves the ROS2 `example_interfaces/Fibonacci` action from a dora Python node
using the `create_action_server` API of the dora ROS2 bridge. Goals are polled
on each dora tick, feedback is streamed per step, and the terminal result is
sent when the sequence is complete.
"""

import pyarrow as pa
from dora import Node, Ros2Context, Ros2NodeOptions, Ros2QosPolicies

# Fibonacci result/feedback is `int32[] sequence`; build the Arrow struct with
# list<int32> so the CDR serialization matches the .action definition.
SEQ_TYPE = pa.struct([("sequence", pa.list_(pa.int32()))])

ros2_context = Ros2Context()
ros2_node = ros2_context.new_node(
    "action_server",
    "/ros2_demo",
    Ros2NodeOptions(rosout=True),
)
action_qos = Ros2QosPolicies(reliable=True, max_blocking_time=0.1)
fibonacci = ros2_node.create_action_server(
    "/fibonacci",
    "example_interfaces/Fibonacci",
    action_qos,
)

dora_node = Node()
handled = 0
for _ in range(2000):
    event = dora_node.next()
    if event is None:
        break
    if event["type"] != "INPUT" or event["id"] != "tick":
        continue

    goal = fibonacci.take_goal(timeout_s=0.05)
    if goal is not None:
        goal_id, value = goal
        order = value[0].as_py()["order"]
        sequence = [0, 1]
        for i in range(1, order):
            sequence.append(sequence[i] + sequence[i - 1])
            fibonacci.send_feedback(goal_id, pa.array([{"sequence": sequence}], type=SEQ_TYPE))
        fibonacci.send_result(
            goal_id,
            pa.array([{"sequence": sequence}], type=SEQ_TYPE),
            status="succeeded",
            timeout_s=20.0,
        )
        handled += 1
        print(f"served fibonacci(order={order}) = {sequence}", flush=True)

    if handled >= 1:
        break

assert handled > 0, "action server handled no goals"
print(f"PYTHON ACTION SERVER OK ({handled} handled)", flush=True)
