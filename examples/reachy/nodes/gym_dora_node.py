import time
from pathlib import Path

import gymnasium as gym
import pandas as pd
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

import gym_dora  # noqa: F401

env = gym.make(
    "gym_dora/DoraReachy2-v0", disable_env_checker=True, max_episode_steps=10000
)
observation = env.reset()


class ReplayPolicy:
    def __init__(self, example_path, epidode=0):
        df_action = pd.read_parquet(example_path / "action.parquet")
        df_episode_index = pd.read_parquet(example_path / "episode_index.parquet")
        self.df = pd.merge_asof(
            df_action[["timestamp_utc", "action"]],
            df_episode_index[["timestamp_utc", "episode_index"]],
            on="timestamp_utc",
            direction="backward",
        )
        # self.df["episode_index"] = self.df["episode_index"].map(lambda x: x[0])
        self.df = self.df[self.df["episode_index"] == epidode]
        self.current_time = self.df["timestamp_utc"].iloc[0]
        self.topic = "action"
        self.index = 0
        self.finished = False

    def select_action(self, obs):
        if self.index < len(self.df):
            self.index += 1
        else:
            self.finished = True
        row = self.df.iloc[self.index]
        delta_time = (row["timestamp_utc"] - self.current_time).microseconds
        self.current_time = row["timestamp_utc"]
        if delta_time > 0:
            time.sleep(delta_time / 1_000_000)
        return row[self.topic], self.finished


class ReplayLeRobotPolicy:
    def __init__(self, episode=21):
        self.index = 0
        self.finished = False
        # episode = 1
        dataset = LeRobotDataset("cadene/reachy2_mobile_base")
        from_index = dataset.episode_data_index["from"][episode]
        to_index = dataset.episode_data_index["to"][episode]
        self.states = dataset.hf_dataset["observation.state"][from_index:to_index]
        self.actions = dataset.hf_dataset["action"][from_index:to_index]

    def select_action(self, obs):
        if self.index < len(self.actions):
            self.index += 1
        else:
            self.finished = True
        # time.sleep(1 / 30)
        return self.actions[self.index].numpy(), self.finished


# policy = ReplayPolicy(
# Path(
# "/home/rcadene/dora-aloha/aloha/graphs/out/018fa076-ad19-7c77-afa4-49f7f072e86f"
# )
# )

policy = ReplayLeRobotPolicy()

done = False
while not done:
    actions, finished = policy.select_action(observation)

    observation, reward, terminated, truncated, info = env.step(actions)
    # cv2.imshow("frame", observation["pixels"]["cam_trunk"])
    # if cv2.waitKey(1) & 0xFF == ord("q"):
    # break
    if terminated:
        print(observation, reward, terminated, truncated, info, flush=True)
    done = terminated | truncated | done | finished

env.close()
