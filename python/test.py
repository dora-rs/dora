from dora_ros2_bridge import *
import time

context = Ros2Context()
node = context.new_node("py_tel", "/", Ros2NodeOptions())

topic_qos = Ros2QosPolicies()
topic_qos.reliability = Ros2Reliability.Reliable
topic_qos.reliability_max_blocking_time = 0.1

turtle_pose_topic = node.create_topic("/turtle1/pose", "turtlesim::msg::dds_::Pose_", topic_qos)
pose_reader = node.create_subscription(turtle_pose_topic, topic_qos)

for i in range(500):
    print(pose_reader.next())
    time.sleep(0.1)
