#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import random
import dora
from dora import Node
import pyarrow as pa

ros2_context = dora.experimental.ros2_bridge.Ros2Context()
ros2_node = ros2_context.new_node(
    "turtle_teleop",
    "/ros2_demo",
    dora.experimental.ros2_bridge.Ros2NodeOptions(rosout=True),
)

topic_qos = dora.experimental.ros2_bridge.Ros2QosPolicies(
    reliable=True, max_blocking_time=0.1
)

turtle_twist_topic = ros2_node.create_topic(
    "/turtle1/cmd_vel", "geometry_msgs::Twist", topic_qos
)
twist_writer = ros2_node.create_publisher(turtle_twist_topic)

turtle_pose_topic = ros2_node.create_topic(
    "/turtle1/pose", "turtlesim::Pose", topic_qos
)
pose_reader = ros2_node.create_subscription(turtle_pose_topic)

dora_node = Node()
dora_node.merge_external_events(pose_reader)

print("looping", flush=True)
for i in range(500):
    event = dora_node.next()
    if event is None:
        break
    match event["kind"]:
        case "dora":
            match event["type"]:
                case "INPUT":
                    match event["id"]:
                        case "direction":
                            direction = {
                                "linear": {
                                    "x": event["value"][0],
                                },
                                "angular": {
                                    "z": event["value"][5],
                                },
                            }

                            print(direction, flush=True)
                            # TODO FIXME: The below line seems to result in an
                            # `["linear", "angular"]` array, completely ignoring the values.
                            direction_arrow = pa.array(
                                direction,
                                # TODO: maybe type annotations help somehow?
                                # type=pa.map_(
                                #     pa.utf8(),
                                #     pa.map_(pa.utf8(), pa.float32()),
                                # ),
                            )
                            print(direction_arrow, flush=True)
                            # twist_writer.publish(direction_arrow)
                        case "tick":
                            pass

        case "external":
            pose = event.inner()[0].as_py()

            assert (
                pose["x"] != 5.544445
            ), "turtle should not be at initial x axis"
            dora_node.send_output(
                "turtle_pose",
                pa.array(
                    [
                        pose["x"],
                        pose["y"],
                        pose["theta"],
                        pose["linear_velocity"],
                        pose["angular_velocity"],
                    ],
                    type=pa.float64(),
                ),
            )
