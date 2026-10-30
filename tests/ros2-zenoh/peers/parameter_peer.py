#!/usr/bin/env python3
# Drives a dora node's parameters with rclpy's own `rcl_interfaces` types, the
# way `ros2 param list/get/set/describe` does, and checks the
# `/parameter_events` it publishes.
import json
import time

import rclpy
from rcl_interfaces.msg import Parameter, ParameterEvent, ParameterType, ParameterValue
from rcl_interfaces.srv import (
    DescribeParameters,
    GetParameters,
    GetParameterTypes,
    ListParameters,
    SetParameters,
    SetParametersAtomically,
)
from rclpy.qos import qos_profile_parameter_events

NODE = "/dora_test/dora_native_peer"

rclpy.init()
n = rclpy.create_node("dora_zenoh_parameter_peer")
events = []
n.create_subscription(
    ParameterEvent, "/parameter_events", events.append, qos_profile_parameter_events
)
clients = {
    name: n.create_client(srv, f"{NODE}/{name}")
    for name, srv in [
        ("list_parameters", ListParameters),
        ("get_parameters", GetParameters),
        ("get_parameter_types", GetParameterTypes),
        ("describe_parameters", DescribeParameters),
        ("set_parameters", SetParameters),
        ("set_parameters_atomically", SetParametersAtomically),
    ]
}
print(json.dumps({"event": "READY"}), flush=True)


def check(ok, what):
    if not ok:
        raise SystemExit(what)


def call(name, request):
    future = clients[name].call_async(request)
    rclpy.spin_until_future_complete(n, future, timeout_sec=10)
    check(future.result() is not None, f"{name}: no response")
    return future.result()


def double(name, value):
    return Parameter(
        name=name,
        value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value),
    )


def speed():
    response = call("get_parameters", GetParameters.Request(names=["speed"]))
    return response.values[0].double_value


def wait_for_event(value):
    end = time.monotonic() + 10
    while time.monotonic() < end:
        for event in events:
            changed = {p.name: p.value.double_value for p in event.changed_parameters}
            if event.node == NODE and changed.get("speed") == value:
                return event
        rclpy.spin_once(n, timeout_sec=0.1)
    raise SystemExit(f"no /parameter_events event for speed={value}: {events!r}")


for client in clients.values():
    check(client.wait_for_service(timeout_sec=20), f"{client.srv_name}: not found")

# ros2 param list
names = call("list_parameters", ListParameters.Request()).result.names
check(sorted(names) == ["gains", "speed", "use_sim_time"], f"list: {names!r}")

# ros2 param get
values = call(
    "get_parameters", GetParameters.Request(names=["speed", "gains", "nope"])
).values
check(values[0].type == ParameterType.PARAMETER_DOUBLE, f"get speed: {values[0]}")
check(values[0].double_value == 1.5, f"get speed: {values[0]}")
check(list(values[1].double_array_value) == [1.0, 2.0], f"get gains: {values[1]}")
check(values[2].type == ParameterType.PARAMETER_NOT_SET, f"get nope: {values[2]}")

# ros2 param describe
types = call(
    "get_parameter_types", GetParameterTypes.Request(names=["speed", "gains"])
).types
check(
    list(types)
    == [ParameterType.PARAMETER_DOUBLE, ParameterType.PARAMETER_DOUBLE_ARRAY],
    f"types: {types!r}",
)
descriptors = call(
    "describe_parameters", DescribeParameters.Request(names=["speed"])
).descriptors
check(
    [(d.name, d.type) for d in descriptors]
    == [("speed", ParameterType.PARAMETER_DOUBLE)],
    f"describe: {descriptors!r}",
)

# ros2 param set: a good value, then a type change the node rejects.
results = call(
    "set_parameters", SetParameters.Request(parameters=[double("speed", 7.5)])
).results
check([r.successful for r in results] == [True], f"set speed: {results!r}")
event = wait_for_event(7.5)
# A `builtin_interfaces/Time` stamp decodes to the current wall time.
check(abs(event.stamp.sec - time.time()) < 60, f"event stamp: {event.stamp}")
check(event.stamp.nanosec < 1_000_000_000, f"event stamp: {event.stamp}")
fast = Parameter(
    name="speed",
    value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value="fast"),
)
results = call("set_parameters", SetParameters.Request(parameters=[fast])).results
check(
    [(r.successful, r.reason) for r in results]
    == [(False, "parameter `speed` is type-stable; cannot change its declared type")],
    f"set speed=fast: {results!r}",
)

# The atomic set replies with one result and changes nothing when one value
# is rejected.
result = call(
    "set_parameters_atomically",
    SetParametersAtomically.Request(parameters=[double("speed", 8.0), double("nope", 1.0)]),
).result
check(not result.successful, f"atomic set with nope: {result!r}")
check(speed() == 7.5, "a rejected atomic set changed speed")
result = call(
    "set_parameters_atomically",
    SetParametersAtomically.Request(parameters=[double("speed", 8.0)]),
).result
check(result.successful, f"atomic set: {result!r}")
wait_for_event(8.0)

# The dora node answers 8.0 by setting 9.0 itself: a local set is published too.
wait_for_event(9.0)
check(speed() == 9.0, "the local set is not visible remotely")
print(json.dumps({"event": "PASS"}), flush=True)
# Tells the dora node it can stop. Its reply doesn't matter any more.
future = clients["set_parameters"].call_async(
    SetParameters.Request(parameters=[double("speed", 10.0)])
)
rclpy.spin_until_future_complete(n, future, timeout_sec=10)
n.destroy_node()
rclpy.shutdown()
