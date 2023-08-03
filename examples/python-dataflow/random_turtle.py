import time, random
from dora_ros2_bridge import *
from dora import Node
import pyarrow as pa

node = Node()
context = Ros2Context()
node = context.new_node("turtle_teleop", "/ros2_demo", Ros2NodeOptions(rosout=True))

topic_qos = Ros2QosPolicies(reliable=True, max_blocking_time=0.1)

turtle_twist_topic = node.create_topic(
    "/turtle1/cmd_vel", "geometry_msgs::Twist", topic_qos
)
twist_writer = node.create_publisher(turtle_twist_topic)

turtle_pose_topic = node.create_topic("/turtle1/pose", "turtlesim::Pose", topic_qos)
pose_reader = node.create_subscription(turtle_pose_topic)

for event in node:
    match event["type"]:
        case "INPUT":
            match event["id"]:
                case "direction":
                    direction = {
                        "linear": {
                            "x": event["data"][0],
                        },
                        "angular": {
                            "z": event["data"][5],
                        },
                    }
                    twist_writer.publish(direction)
                case "tick":
                    pose = pose_reader.next()
                    if pose == None:
                        break
                    node.send_output(
                        pa.array(
                            [
                                pose["x"],
                                pose["y"],
                                pose["z"],
                                pose["theta"],
                                pose["linar_velocity"],
                                pose["angular_velocity"],
                            ],
                            type=pa.uint8(),
                        )
                    )
