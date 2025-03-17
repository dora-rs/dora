import dora
import numpy as np
import pyarrow as pa
from dora import Node

ros2_context = dora.experimental.ros2_bridge.Ros2Context()
ros2_node = ros2_context.new_node(
    "robot_model_master",
    "/dora",
    dora.experimental.ros2_bridge.Ros2NodeOptions(rosout=True),
)

# Define a ROS2 QOS
topic_qos = dora.experimental.ros2_bridge.Ros2QosPolicies(
    reliable=True, max_blocking_time=0.1,
)

# Create a publisher to cmd_vel topic
puppet_arm_command = ros2_node.create_publisher(
    ros2_node.create_topic(
        "/robot_model_puppet/commands/joint_group",
        "interbotix_xs_msgs/JointGroupCommand",
        topic_qos,
    ),
)

gripper_command = ros2_node.create_publisher(
    ros2_node.create_topic(
        "/robot_model_puppet/commands/joint_single",
        "interbotix_xs_msgs/JointSingleCommand",
        topic_qos,
    ),
)
# Create a listener to pose topic
master_state = ros2_node.create_subscription(
    ros2_node.create_topic(
        "/robot_model_master/joint_states", "sensor_msgs/JointState", topic_qos,
    ),
)

# Create a dora node
dora_node = Node()

# Listen for both stream on the same loop as Python does not handle well multiprocessing
dora_node.merge_external_events(master_state)

PUPPET_GRIPPER_MAX = 0.115
PUPPET_GRIPPER_MIN = 0.0965

MASTER_GRIPPER_MAX = 0.8
MASTER_GRIPPER_MIN = -0.1

RATIO = (PUPPET_GRIPPER_MAX - PUPPET_GRIPPER_MIN) / (
    MASTER_GRIPPER_MAX - MASTER_GRIPPER_MIN
)

for event in dora_node:
    event_kind = event["kind"]

    # ROS2 Event
    if event_kind == "external":
        pose = event.inner()[0]

        values = np.array(pose["position"].values, dtype=np.float32)
        values[6] = np.clip(
            (values[6] - MASTER_GRIPPER_MIN) * RATIO + PUPPET_GRIPPER_MIN,
            PUPPET_GRIPPER_MIN,
            PUPPET_GRIPPER_MAX,
        )
        gripper_command.publish(
            pa.array(
                [
                    {
                        "name": "gripper",
                        "cmd": np.float32(values[6]),
                    },
                ],
            ),
        )
        puppet_arm_command.publish(
            pa.array(
                [
                    {
                        "name": "arm",
                        "cmd": values[:6],
                    },
                ],
            ),
        )
