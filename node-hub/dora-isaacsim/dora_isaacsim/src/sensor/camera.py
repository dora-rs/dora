# import rospy
from omni.isaac.sensor import Camera as IsaacCamera
# from sensor_msgs.msg import Image
# from std_msgs.msg import Header
import pyarrow as pa
# from .base import BaseSensor


class Camera(IsaacCamera):
    def __init__(
        self, camera_prim_path, sim_freq, sensor_freq, resolution, queue_size=10, *args, **kwargs
    ):
        super().__init__(
            prim_path=camera_prim_path,  # 使用 USD 中的路径，不重新定义
            frequency=sensor_freq,
            resolution=(resolution[0], resolution[1]),
            *args,
            **kwargs,
        )

        self.sim_dt = 1 / sim_freq
        self.sensor_dt = 1 / sensor_freq
        self.sim_time = 0

    def reset(self):
        self.sim_time = 0

    def spawn(self, *args):
        self.initialize()

    def get_data(self):
        rgb_data = self.get_rgb()  # 获取 RGB 数据
        return rgb_data

    def send_dora_message(self, node, seq=None, time_stamp=None, frame_id="sim_camera", encoding="rgb8"):
        self.sim_time += self.sim_dt
        if self.sim_time >= self.sensor_dt:
            self.sim_time -= self.sensor_dt
            image_np = self.get_data()

            if len(image_np.shape) == 3:
                height = image_np.shape[0]
                width = image_np.shape[1]
                num_channels = image_np.shape[2]
            elif len(image_np.shape) == 2:  # Grayscale
                height = image_np.shape[0]
                width = image_np.shape[1]
                num_channels = 1
            else:
                raise ValueError("Image NumPy array must be 2D (grayscale) or 3D (color).")
            
            node.send_output(
                output_id="camera", 
                data=pa.array(image_np.flatten()), 
                metadata={"w": width, "h": height, "seq": seq, "stamp": time_stamp, "frame_id": frame_id, "encoding": encoding, "is_bigendian": 0, "step": width*num_channels*image_np.itemsize}
            )
