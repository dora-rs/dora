#!/usr/bin/env python

from dora import Node, Ros2Context, Ros2NodeOptions, Ros2QosPolicies

CHECK_TICK = 50

# Create a ROS2 Context
ros2_context = Ros2Context()
ros2_node = ros2_context.new_node(
    "turtle_teleop",
    "/ros2_demo",
    Ros2NodeOptions(rosout=True),
)

# Define a ROS2 QOS
topic_qos = Ros2QosPolicies(reliable=True, max_blocking_time=0.1)

# Create a publisher to cmd_vel topic
turtle_twist_topic = ros2_node.create_topic(
    "/turtle1/cmd_vel", "geometry_msgs/Twist", topic_qos,
)
twist_writer = ros2_node.create_publisher(turtle_twist_topic)

# Create a listener to pose topic
turtle_pose_topic = ros2_node.create_topic("/turtle1/pose", "turtlesim/Pose", topic_qos)
pose_reader = ros2_node.create_subscription(turtle_pose_topic)

# Create a dora node
dora_node = Node()

# Listen for both stream on the same loop as Python does not handle well multiprocessing
dora_node.merge_external_events(pose_reader)

print("looping", flush=True)

# take track of minimum and maximum coordinates of turtle
min_x = 1000
max_x = 0
min_y = 1000
max_y = 0

for i in range(500):
    event = dora_node.next()
    if event is None:
        break
    event_kind = event["kind"]
    # Dora event
    if event_kind == "dora":
        event_type = event["type"]
        if event_type == "INPUT":
            event_id = event["id"]
            if event_id == "direction":
                twist_writer.publish(event["value"])

        # ROS2 Event
    elif event_kind == "external":
        pose = event["value"][0].as_py()
        min_x = min([min_x, pose["x"]])
        max_x = max([max_x, pose["x"]])
        min_y = min([min_y, pose["y"]])
        max_y = max([max_y, pose["y"]])
        dora_node.send_output("turtle_pose", event["value"])

assert max_x - min_x > 1 or max_y - min_y > 1, "no turtle movement"
