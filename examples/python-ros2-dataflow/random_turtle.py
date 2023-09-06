#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import random
import dora
from dora import Node
import pyarrow as pa

CHECK_TICK = 20

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
                            twist_writer.publish(event["value"])

        case "external":
            pose = event.inner()[0].as_py()
            if i == CHECK_TICK:
                assert (
                    pose["x"] != 5.544444561004639
                ), "turtle should not be at initial x axis"
            dora_node.send_output("turtle_pose", event.inner())
