### Task parameters
DATA_DIR = "data/input"
IMG_H = 360
IMG_W = 640
TASK_CONFIGS = {
    "grasp_corn": {
        "dataset_dir": DATA_DIR + "/grasp_corn_joint_for_real_v0",
        "num_episodes": 68,
        "episode_len": 180,
        "camera_names": ["front"],
        "state_dim": 8,
    },
    "grasp_cube": {
        "dataset_dir": DATA_DIR + "/grasp_cube_joint_for_real_v0",
        "num_episodes": 81,
        "episode_len": 180,
        "camera_names": ["front"],
        "state_dim": 8,
    },
    "stack_cube": {
        "dataset_dir": "./dataset/stack_cube",  # tip: 这里的路径要改成你自己的数据集的路径
        "num_episodes": 15,  # 实际采集的 eposide 的数量
        "episode_len": 125,  # 每个 episode 的长度
        "camera_names": ["front"],
        "state_dim": 8,
    },
}
