#!/usr/bin/env python
"""dora node exposing ROS2 parameters via the bridge.

Declares parameters on the ROS2 node `/demo_params` and exercises the parameter
API. The six rcl_interfaces parameter services are hosted natively by ros2-client
(driven by the node spinner), so `ros2 param` / rclpy clients can query them over
DDS (subject to ros2-client#4 discoverability); this node uses the local
get/set/list API, which needs no discovery and is therefore deterministic.
"""

from dora import Node, Ros2Context, Ros2NodeOptions

# Connect to the dora daemon first (before the slower ROS2 setup), so the daemon
# does not kill this node for not initializing its connection in time.
dora_node = Node()

ros2_context = Ros2Context()
ros2_node = ros2_context.new_node(
    "demo_params",
    "/",
    Ros2NodeOptions(rosout=True),
    parameters={
        "speed": 1.5,
        "name": "robot",
        "gain": 7,
        "waypoints": [1.0, 2.0, 3.0],
        "flags": [True, False],
    },
)

# Local parameter API (no cross-node discovery needed).
print("list_parameters:", ros2_node.list_parameters(), flush=True)
assert ros2_node.has_parameter("speed")
assert ros2_node.get_parameter("speed") == 1.5
assert ros2_node.get_parameter("name") == "robot"
assert ros2_node.get_parameter("gain") == 7
assert ros2_node.get_parameter("waypoints") == [1.0, 2.0, 3.0]
assert ros2_node.get_parameter("flags") == [True, False]
assert ros2_node.get_parameter("missing") is None

# Update a parameter (same type) and read it back.
ros2_node.set_parameter("speed", 2.0)
assert ros2_node.get_parameter("speed") == 2.0

# Changing a declared parameter's type is rejected (type-stable contract,
# enforced by the validator on every set path including remote `ros2 param`).
try:
    ros2_node.set_parameter("gain", "not an int")
    raise AssertionError("expected a type-change rejection for `gain`")
except RuntimeError:
    pass

# `use_sim_time` is a built-in and cannot be declared (ros2-client would
# silently overwrite it); declaring it must be rejected.
try:
    ros2_context.new_node(
        "bad_sim_time", "/", Ros2NodeOptions(), parameters={"use_sim_time": True}
    )
    raise AssertionError("expected `use_sim_time` declaration to be rejected")
except RuntimeError:
    pass

# A non-homogeneous list is rejected rather than silently coerced -- including a
# bool mixed into an int list (`bool` is an `int` subclass in Python).
for bad in ([1, "two"], [1, True]):
    try:
        ros2_context.new_node(
            "bad_list", "/", Ros2NodeOptions(), parameters={"mixed": bad}
        )
        raise AssertionError(f"expected non-homogeneous list {bad!r} to be rejected")
    except RuntimeError:
        pass

print("PARAM API OK", flush=True)

# Keep the node (and its parameter services) alive briefly so an external ROS2
# client could query it; the example asserts above are the smoke contract.
for _ in range(50):
    event = dora_node.next()
    if event is None:
        break
