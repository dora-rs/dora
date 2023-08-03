from dora_ros2_bridge import *
import time, random

context = Ros2Context()
node = context.new_node("turtle_teleop", "/ros2_demo", Ros2NodeOptions(rosout = True))

topic_qos = Ros2QosPolicies(reliable = True, max_blocking_time = 0.1)

turtle_twist_topic = node.create_topic("/turtle1/cmd_vel", "geometry_msgs::Twist", topic_qos)
twist_writer = node.create_publisher(turtle_twist_topic)

turtle_pose_topic = node.create_topic("/turtle1/pose", "turtlesim::Pose", topic_qos)
pose_reader = node.create_subscription(turtle_pose_topic)

for i in range(500):
    direction = {
        'linear': {
            'x': random.random() + 1,
        },
        'angular': {
            'z': (random.random() - 0.5) * 5,
        }
    }
    twist_writer.publish(direction);
    while True:
        pose = pose_reader.next()
        if pose == None:
            break
        print(pose)
    time.sleep(0.5)
