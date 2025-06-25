from src.utils import draw_single_point


class BaseTask:
    def __init__(
        self,
        scenary,
        robot,  # A dictory contains many robots
        sensors={},
        dataset=None,
        replay_horizon=0,
        replay_trajectory_index=0,
        visualize_trajectory=False,
    ):
        self.scenary = scenary
        self.robot = robot
        self.sensors = sensors

        self.reset_needed = False  # set to true if you restart the simulator
        self.dataset = dataset
        self.replay_horizon = (
            replay_horizon if replay_horizon != -1 else self.dataset[0]["action"].shape[0]
        )
        self.replay_trajectory_index = replay_trajectory_index

        self.done_flag = False
        # build task
        self.build()
        self.replay_count = 0
        self.visualize_trajectory = visualize_trajectory

    def reset(self):
        self.scenary.reset()
        self.robot.reset()
        for _, sensor in self.sensors.items():
            sensor.reset()

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

    def step(self, reset):
        # 获取动作
        next_action = self.robot.compute_action()

        # 控制机器人
        self.robot.apply_action(next_action)

    def send_ros_data(self, seq, time_stamp):
        for name, sensor in self.sensors.items():
            sensor.send_ros_message(seq=seq, time_stamp=time_stamp)
    
    def send_dora_data(self, seq, time_stamp):
        for name, sensor in self.sensors.items():
            sensor.send_dora_message(self.robot.controller.node, seq=seq, time_stamp=time_stamp)

    def run(self, simulation_app):
        world = self.scenary.world
        warm_up = 100
        i = 0
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

                self.send_dora_data(seq=i, time_stamp=world.current_time)
                if self.visualize_trajectory:
                    ee_pose = self.robot.get_ee_pose()
                    draw_single_point(ee_pose[:3], color="model")

                self.step(reset)

                i = i + 1

                if self.done_flag:
                    break
        simulation_app.close()
