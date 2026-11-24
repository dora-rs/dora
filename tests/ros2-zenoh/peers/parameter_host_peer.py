#!/usr/bin/env python3
# An rclpy node with parameters that a dora parameter client reads and tunes
# (#4297). It sets `speed` to 4.0 itself once dora has set it to 3.0, so dora
# also sees a change the node made, and passes once dora sets 5.0 ("done").
import json
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.parameter import Parameter

rclpy.init()
n = rclpy.create_node("arm", namespace="/robot")
n.declare_parameter("speed", 1.5, ParameterDescriptor(description="speed in m/s"))
n.declare_parameter("gains", [1.0, 2.0])


def no_negative_speed(parameters):
    for parameter in parameters:
        if parameter.name == "speed" and parameter.value < 0:
            return SetParametersResult(successful=False, reason="speed must not be negative")
    return SetParametersResult(successful=True)


n.add_on_set_parameters_callback(no_negative_speed)
print(json.dumps({"event": "READY"}), flush=True)

end = time.monotonic() + 50
answered = False
while time.monotonic() < end:
    rclpy.spin_once(n, timeout_sec=0.1)
    speed = n.get_parameter("speed").value
    if speed == 3.0 and not answered:
        n.set_parameters([Parameter("speed", value=4.0)])
        answered = True
    if speed == 5.0:
        # Let the reply to dora's last set leave.
        end_settle = time.monotonic() + 1
        while time.monotonic() < end_settle:
            rclpy.spin_once(n, timeout_sec=0.1)
        gains = list(n.get_parameter("gains").value)
        if gains != [3.0]:
            raise SystemExit(f"gains {gains!r}, expected [3.0] from the atomic set")
        print(json.dumps({"event": "PASS"}), flush=True)
        break
else:
    raise SystemExit(f"dora never finished: speed={n.get_parameter('speed').value}")
n.destroy_node()
rclpy.shutdown()
