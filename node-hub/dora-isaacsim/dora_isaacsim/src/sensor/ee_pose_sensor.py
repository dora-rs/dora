import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from omni.isaac.core.prims import XFormPrim
import pyarrow as pa
from .base import BaseSensor


class EEPoseSensor(BaseSensor):
    def __init__(self, robot_name, ee_prim_path, queue_size=10, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.robot_name = robot_name
        self.ee_prim_path = ee_prim_path
        # rospy.init_node(name, anonymous=True)
        topic_name = "/" + self.robot_name + "_ee_pose"  # 拼接 topic 名
        self.ee_pose_pub = rospy.Publisher(topic_name, PoseStamped, queue_size=queue_size)

    def reset(self):
        pass

    def spawn(self, world):
        pass

    def get_data(self):
        ee_prim = XFormPrim(self.ee_prim_path)
        state = ee_prim.get_default_state()
        return np.concatenate([state.position, state.orientation], axis=-1)

    def send_ros_message(self, seq=None, time_stamp=None):
        ee_pose = self.get_data()  # 假设 ee_pose 是一个包含 position 和 orientation 的对象/dict

        msg = PoseStamped()
        msg.header.seq = seq
        msg.header.stamp = rospy.Time.from_sec(time_stamp)
        msg.header.frame_id = "world"  # 可根据需要设置坐标系

        # 设定位置
        msg.pose.position.x = ee_pose[0]
        msg.pose.position.y = ee_pose[1]
        msg.pose.position.z = ee_pose[2]

        # 设定姿态（wxyz四元数）
        msg.pose.orientation.w = ee_pose[3]
        msg.pose.orientation.x = ee_pose[4]
        msg.pose.orientation.y = ee_pose[5]
        msg.pose.orientation.z = ee_pose[6]
        self.ee_pose_pub.publish(msg)
    
    def send_dora_message(self, node, seq=None, time_stamp=None):
        ee_pose = self.get_data()
        node.send_output(
            output_id="ee_pose", 
            data=pa.array(ee_pose), 
            metadata={"seq":seq, "stamp": time_stamp, "frame_id": "world"}
        )
