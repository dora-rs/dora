import queue

# import rospy
# from act_dp_service.msg import DualArmState, SingleArmState
# from act_dp_service.srv import get_action, get_action_bimanual
# from std_msgs.msg import (
#     Float64,
#     Float64MultiArray,
#     MultiArrayDimension,
#     MultiArrayLayout,
#     UInt8MultiArray,
# )

from .base import BaseController
from dora import Node


class DoraSubscriberController(BaseController):
    def __init__(self, sync=False, queue_size=1, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.queue_size = queue_size
        self.action_queue = queue.Queue(maxsize=queue_size)
        self.new_available = False
        # self.subscribe_topic_name = subscribe_topic_name
        self.sync = sync

        self.node = Node()

        # 检查 topic 是否存在
        # published_topics = rospy.get_published_topics()
        # topic_names = [t[0] for t in published_topics]

        # if self.subscribe_topic_name not in topic_names:
        #     raise ValueError(
        #         f"Topic '{self.subscribe_topic_name}' does not exist. "
        #         "Please make sure the publisher is running."
        #     )

        # rospy.Subscriber(subscribe_topic_name, Float64MultiArray, self._action_callback)

    # def _action_callback(self, msg):
    #     action_data = np.array(msg.data, dtype=np.float64)
    #     try:
    #         self.action_queue.put_nowait(action_data)  # <--- 向 queue.Queue 中放入数据
    #     except queue.Full:
    #         rospy.logwarn(
    #             f"Action queue is full for topic '{self.subscribe_topic_name}'. Discarding new action."
    #         )

    def _action_callback(self):
        
        event = self.node.next(0.01)
        if event is not None and event["type"] == "INPUT" and event["id"] == "action":
            action_data = event["value"].to_pylist()
            try:
                self.action_queue.put_nowait(action_data)
            except queue.Full:
                pass

    def forward(self):
        self._action_callback()
        self.sim_time += self.sim_dt
        if self.sim_time >= self.control_dt:
            self.sim_time -= self.control_dt
            if not self.sync:
                try:
                    next_action = self.action_queue.get_nowait()
                except queue.Empty:  # 如果队列为空
                    return None
            else:
                next_action = self.action_queue.get(block=True)  # 如果队列中没有数据则一直阻塞
            return next_action


# class DualArmROSController(ROSSubscriberController):
#     def process_proprioception(self, observation):
#         # 处理机器人的本体数据
#         ee_length = observation["ee_pose"].shape[0] // 2
#         joint_length = observation["joint_pos"].shape[0] // 2
#         states = {}
#         for arm_side in ["left", "right"]:
#             if arm_side == "left":
#                 s = slice(None, ee_length)
#                 s2 = slice(None, joint_length)
#             else:
#                 s = slice(ee_length, None)
#                 s2 = slice(joint_length, None)

#             ee_pose = observation["ee_pose"][s]
#             ee_pose = Float64MultiArray(data=list(ee_pose))

#             joint_pos = observation["joint_pos"][s2]
#             joint_pos = Float64MultiArray(data=list(joint_pos))

#             gripper_width = Float64(data=observation["joint_pos"][s2][-1])
#             arm_state = SingleArmState(
#                 ee_pose,
#                 joint_pos,
#                 gripper_width,
#             )

#             states[arm_side] = arm_state
#         return states

#     def get_action_from_ros(self, states, rgb_data, reset):
#         rospy.wait_for_service(self.ros_service_name)
#         get_control_action = rospy.ServiceProxy(self.ros_service_name, get_action_bimanual)
#         target_action = get_control_action(states=states, font_camera=rgb_data, reset=reset)
#         return target_action

#     def forward(self, observation):
#         rgb_data = self.process_rgb_data(observation["env_sensors"]["camera_rgb_front"])
#         arm_states_dict = self.process_proprioception(observation["robot"]["states"])
#         left_arm_state = arm_states_dict["left"]
#         right_arm_states = arm_states_dict["right"]
#         arm_states = DualArmState(left_arm_state, right_arm_states)
#         reset = Float64(data=observation["reset"])
#         target_action = self.get_action_from_ros(arm_states, rgb_data, reset)
#         return np.array(target_action.actions.data)
