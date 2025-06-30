# import rospy
import pyarrow as pa
from .base import BaseSensor


class JointSensor(BaseSensor):
    def __init__(self, robot_name, articulation=None, queue_size=10, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.name = robot_name + "_joint"
        self.robot_name = robot_name
        self.articulation = articulation

    def reset(self):
        pass

    def spawn(self, world):
        robot = world.scene.get_object(self.robot_name)
        self.articulation = robot

    def get_data(self):
        states = self.articulation.get_joints_state()
        return states
    
    def send_dora_message(self, node, seq=None, time_stamp=None):
        joint_state = self.get_data()
        node.send_output(
            output_id="joint_pos", 
            data=pa.array(joint_state.positions), 
            metadata={"seq": seq, "stamp": time_stamp, "name": self.articulation.dof_names}
        )
