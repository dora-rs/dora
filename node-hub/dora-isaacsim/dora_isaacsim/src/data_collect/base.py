import os

from PIL import Image


class BaseDataCollect:
    def __init__(
        self,
        scenary,
        robot,
        sensors={},
        save_dir="./output_data",
        save_trajectory=True,
        *args,
        **kwargs,
    ):
        self.scenary = scenary
        self.robot = robot
        self.sensors = sensors
        self.save_trajectory = save_trajectory
        self.img_count = 0
        self.save_dir = self.get_next_episode_dir(save_dir)
        os.makedirs(self.save_dir, exist_ok=True)

        self.reset_needed = False  # set to true if you restart the simulator
        self.done_flag = False
        # build task
        self.build()

        self.collected_data = []

    def build(self):
        # 1. 加载场景
        self.scenary.load_stage()

        # 2. 创建机器人
        self.robot.spawn(self.scenary.world)

        # 3. 创建传感器
        for _, sensor in self.sensors.items():
            sensor.spawn(self.scenary.world)

        # 5. 重置环境
        self.reset()

    def get_raw_data(self):
        data = {"sensors": {}, "robot": {}, "action": {}}
        # 获取传感器数据
        for name, sensor in self.sensors.items():
            data["sensors"][name] = sensor.get_data()

        # 获取机器人本体数据
        robot_states = self.robot.get_states()

        data["robot"]["states"] = robot_states

        return data

    def get_next_episode_dir(self, base_dir):
        """在 `base_dir` 下创建一个新的 `epo_xx` 目录，编号递增"""
        os.makedirs(base_dir, exist_ok=True)  # 确保基础目录存在
        existing_dirs = [
            d for d in os.listdir(base_dir) if d.startswith("epo_") and d[4:].isdigit()
        ]
        existing_dirs.sort(key=lambda x: int(x[4:]))  # 按编号排序

        if existing_dirs:
            last_index = int(existing_dirs[-1][4:])  # 获取最后一个编号
            new_index = last_index + 1
        else:
            new_index = 0  # 第一次运行，从 0 开始

        new_episode_dir = os.path.join(base_dir, f"epo_{new_index}")
        os.makedirs(new_episode_dir)  # 创建新目录
        return new_episode_dir

    def reset(self):
        self.scenary.reset()
        self.robot.reset()
        for _, sensor in self.sensors.items():
            sensor.reset()

    def save_data(self, *args, **kwargs):
        pass

    def step(self, reset):
        # 获取传感器数据
        data = self.get_raw_data()

        # 获取动作
        next_action = self.robot.compute_action()

        if self.save_trajectory:
            # 直接存储专家轨迹作为 action
            trajectory = self.robot.get_trajectory()
            data["action"]["ee_pose"] = trajectory
            data["action"]["joint_pos"] = next_action

        self.collected_data.append(data)
        if self.robot.controller.is_done():
            print("done picking and placing")
            self.save_data(
                self.save_dir,
                self.collected_data,
            )
            self.done_flag = True

        # 控制机器人
        self.robot.apply_action(next_action)

        # 保存采集的图像

        for name, sensor_data in data["sensors"].items():
            if "camera" in name or "img" in name or "image" in name:
                self.save_img(sensor_data, name)

        self.img_count += 1

    def run(self, simulation_app):
        world = self.scenary.world
        warm_up = 100
        i = 0
        step_count = 0
        while simulation_app.is_running():
            # 推进仿真并渲染
            self.scenary.step(render=True)
            if world.is_stopped() and not self.reset_needed:
                self.reset_needed = True
            if i < warm_up:
                i += 1
                continue
            if world.is_playing():
                reset = self.reset_needed
                if self.reset_needed:
                    self.reset()
                    self.replay_count = 0
                    i = 0
                    self.reset_needed = False

                self.step(reset)
                step_count += 1

                i = i + 1

                if self.done_flag:
                    break

        simulation_app.close()

    def save_img(self, rgb_data, sensor_name="camera"):
        output_dir = self.save_dir
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        camera_dir = os.path.join(output_dir, "camera")
        if not os.path.exists(camera_dir):
            os.makedirs(camera_dir)

        img = Image.fromarray(rgb_data)  # 展平

        file_path = os.path.join(camera_dir, f"{sensor_name}_{self.img_count}.png")
        if os.path.exists(file_path):
            print(f"Frame {file_path} already recorded. Skipping save.")
        else:
            img.save(file_path)
