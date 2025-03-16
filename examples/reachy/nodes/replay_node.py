from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
import time
from dora import Node
import pyarrow as pa

episode = 1
dataset = LeRobotDataset("cadene/reachy2_teleop_remi")
from_index = dataset.episode_data_index["from"][episode]
to_index = dataset.episode_data_index["to"][episode]
actions = dataset.hf_dataset["action"][from_index:to_index]
states = dataset.hf_dataset["observation.state"][from_index:to_index]

images = dataset[from_index.item()]["observation.images.cam_trunk"]


node = Node()

time.sleep(1)
for state in states:
    node.send_output("agent_pos", pa.array(state.numpy()))
    time.sleep(1 / 30)
