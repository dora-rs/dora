#!/usr/bin/env python
"""Python ROS2 action client example.

Sends a goal to the ROS2 `example_interfaces/Fibonacci` action (provided by
the `examples_rclcpp_minimal_action_server` node, or by the dora action-server
example) from a dora Python node, using the `create_action_client` API of the
dora ROS2 bridge. Streams feedback and blocks for the terminal result.
"""

import pyarrow as pa
from dora import Node, Ros2Context, Ros2NodeOptions, Ros2QosPolicies

# Fibonacci goal is `int32 order`; build the Arrow struct with int32 so the
# CDR serialization matches the .action definition (pyarrow defaults to int64).
GOAL_TYPE = pa.struct([("order", pa.int32())])
EXPECTED = [0, 1, 1, 2, 3, 5]  # fibonacci(order=5)

ros2_context = Ros2Context()
ros2_node = ros2_context.new_node(
    "action_client",
    "/ros2_demo",
    Ros2NodeOptions(rosout=True),
)

# ROS2 actions (services underneath) require reliable QoS.
action_qos = Ros2QosPolicies(reliable=True, max_blocking_time=0.1)
fibonacci = ros2_node.create_action_client(
    "/fibonacci",
    "example_interfaces/Fibonacci",
    action_qos,
)

dora_node = Node()
done = False
for _ in range(60):
    event = dora_node.next()
    if event is None:
        break
    if event["type"] != "INPUT" or event["id"] != "tick" or done:
        continue

    # There is no wait_for_action_server in ros2-client 0.8, so send_goal may
    # time out until the server is discovered; retry on the next tick.
    try:
        goal_id = fibonacci.send_goal(pa.array([{"order": 5}], type=GOAL_TYPE), timeout_s=5.0)
    except RuntimeError as err:
        print(f"action server not ready yet ({err}); retrying", flush=True)
        continue
    if goal_id is None:
        print("goal rejected; retrying", flush=True)
        continue
    print(f"client: goal accepted {goal_id}", flush=True)

    # Drain a few feedback messages (best effort), then block for the terminal
    # result. take_result issues a single GetResult request and awaits it, so
    # the result is delivered exactly once.
    for _ in range(10):
        feedback = fibonacci.take_feedback(goal_id, timeout_s=0.1)
        if feedback is not None:
            print(f"feedback: {feedback[0].as_py()['sequence']}", flush=True)

    result = fibonacci.take_result(goal_id, timeout_s=30.0)
    if result is not None:
        status, value = result
        sequence = value[0].as_py()["sequence"]
        print(f"result ({status}): {sequence}", flush=True)
        assert status == "succeeded", f"unexpected status {status}"
        assert sequence == EXPECTED, f"expected {EXPECTED}, got {sequence}"
        done = True
        break

assert done, "no action result received"
print("PYTHON ACTION CLIENT OK", flush=True)
