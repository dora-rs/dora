import time, random
from dora import Node
from dora_ros2_bridge import *
import pyarrow as pa


dora_node = Node()
context = Ros2Context()
node = context.new_node("turtle_teleop", "/ros2_demo", Ros2NodeOptions(rosout=True))

topic_qos = Ros2QosPolicies(reliable=True, max_blocking_time=0.1)

turtle_twist_topic = node.create_topic(
    "/turtle1/cmd_vel", "geometry_msgs::Twist", topic_qos
)
twist_writer = node.create_publisher(turtle_twist_topic)

turtle_pose_topic = node.create_topic("/turtle1/pose", "turtlesim::Pose", topic_qos)
pose_reader = node.create_subscription(turtle_pose_topic)

print("looping", flush=True)
for i in range(500):
    event = dora_node.next()
    if event is None:
        break
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

                    print(direction, flush=True)
                    assert direction["linear"]["x"], "direction should not be 0."
                    twist_writer.publish(direction)
                case "tick":
                    pose = (
                        pose_reader.next()
                    )  # TODO: Replace when stream merge is merged

                    if pose == None:
                        print("stop", flush=True)
                        continue

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
                            type=pa.uint8(),
                        ),
                    )
