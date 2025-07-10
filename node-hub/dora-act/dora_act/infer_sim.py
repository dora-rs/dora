import argparse
import os
import pickle

import numpy as np
import torch
from einops import rearrange
from policy import ACTPolicy, CNNMLPPolicy

from inference import ActInference
import pyarrow as pa

class ActInferenceSim(ActInference):
    def __init__(self):
        super().__init__()
        self.parser.add_argument("--eval", action="store_true")
        self.parser.add_argument("--ckpt_path", type=str, default="policy_best.ckpt")
        args = vars(self.parser.parse_args())

        self.STATE_DIM = 8
        self.exp_weight = 0.1
        self.is_degree = False

        # command line parameters
        is_eval = args["eval"]
        ckpt_dir = args["ckpt_dir"]
        policy_class = args["policy_class"]
        onscreen_render = args["onscreen_render"]
        task_name = args["task_name"]
        batch_size_train = args["batch_size"]
        batch_size_val = args["batch_size"]
        num_epochs = args["num_epochs"]

        # get task parameters
        is_sim = task_name[:4] == "sim_"
        if is_sim:
            from constants import SIM_TASK_CONFIGS

            task_config = SIM_TASK_CONFIGS[task_name]
        else:
            from constants import TASK_CONFIGS  # TODO

            task_config = TASK_CONFIGS[task_name]
        dataset_dir = task_config["dataset_dir"]
        num_episodes = task_config["num_episodes"]
        episode_len = task_config["episode_len"]
        state_dim = task_config["state_dim"]
        camera_names = task_config["camera_names"]

        lr_backbone = 1e-5
        backbone = "resnet18"
        if policy_class == "ACT":
            enc_layers = 4
            dec_layers = 7
            nheads = 8
            policy_config = {
                "state_dim": state_dim,
                "lr": args["lr"],
                "num_queries": args["chunk_size"],
                "kl_weight": args["kl_weight"],
                "hidden_dim": args["hidden_dim"],
                "dim_feedforward": args["dim_feedforward"],
                "lr_backbone": lr_backbone,
                "backbone": backbone,
                "enc_layers": enc_layers,
                "dec_layers": dec_layers,
                "nheads": nheads,
                "camera_names": camera_names,
            }
        elif policy_class == "CNNMLP":
            policy_config = {
                "lr": args["lr"],
                "lr_backbone": lr_backbone,
                "backbone": backbone,
                "num_queries": 1,
                "camera_names": camera_names,
            }
        else:
            raise NotImplementedError

        self.config = {
            "num_epochs": num_epochs,
            "ckpt_dir": ckpt_dir,
            "ckpt_name": args["ckpt_path"],
            "episode_len": episode_len,
            "state_dim": state_dim,
            "lr": args["lr"],
            "policy_class": policy_class,
            "onscreen_render": onscreen_render,
            "policy_config": policy_config,
            "task_name": task_name,
            "seed": args["seed"],
            "temporal_agg": args["temporal_agg"],
            "camera_names": camera_names,
            "real_robot": not is_sim,
        }

    def make_policy(self, policy_class, policy_config):
        if policy_class == "ACT":
            policy = ACTPolicy(policy_config)
        elif policy_class == "CNNMLP":
            policy = CNNMLPPolicy(policy_config)
        else:
            raise NotImplementedError
        return policy
    
    def make_optimizer(self, policy_class, policy):
        if policy_class == "ACT":
            optimizer = policy.configure_optimizers()
        elif policy_class == "CNNMLP":
            optimizer = policy.configure_optimizers()
        else:
            raise NotImplementedError
        return optimizer
    
    def get_image(self, request=True):
        got_image = False
        while not got_image:
            if request:
                self.node.send_output(output_id="request_image", data=pa.array([]), metadata={})
            event = self.node.next(1)
            if event is not None and event["type"] == "INPUT" and event["id"] == "image":
                self.image = event["value"].to_numpy()
                metadata = event["metadata"]
                self.IMAGE_WIDTH = metadata['w']
                self.IMAGE_HEIGHT = metadata['h']
                got_image = True
    
    def process_image(self, image):
        image = rearrange(image, "h w c -> c h w")  # 形状变为 (c, h, w)

        # 转换为 PyTorch 张量，归一化，并移动到 GPU 上
        image = torch.from_numpy(image / 255.0).float().cuda().unsqueeze(0)  # 添加批量维度

        return image

    def inference(self):
        ckpt_name = self.config["ckpt_name"]

        ckpt_dir = self.config["ckpt_dir"]
        state_dim = self.config["state_dim"]
        policy_class = self.config["policy_class"]
        policy_config = self.config["policy_config"]
        max_timesteps = self.config["episode_len"]
        temporal_agg = self.config["temporal_agg"]

        # load policy and stats
        ckpt_path = os.path.join(ckpt_dir, ckpt_name)
        self.policy = self.make_policy(policy_class, policy_config)
        loading_status = self.policy.load_state_dict(torch.load(ckpt_path))
        print(loading_status)
        self.policy.cuda()
        self.policy.eval()
        print(f"Loaded: {ckpt_path}")
        stats_path = os.path.join(ckpt_dir, f"dataset_stats.pkl")
        with open(stats_path, "rb") as f:
            stats = pickle.load(f)

        pre_process = lambda s_qpos: (s_qpos - stats["qpos_mean"]) / stats["qpos_std"]
        post_process = lambda a: a * stats["action_std"] + stats["action_mean"]

        # load environment

        query_frequency = policy_config["num_queries"]  # num_queries == chunk_size
        if temporal_agg:  # temporal aggregation
            query_frequency = 1
            num_queries = policy_config["num_queries"]

        max_timesteps = int(2 * max_timesteps * 1)  # may increase for real-world tasks


        if temporal_agg:
            all_time_actions = torch.zeros([max_timesteps, max_timesteps + num_queries, state_dim]).cuda()

        qpos_history = torch.zeros((1, max_timesteps, state_dim)).cuda()
        image_list = []  # for visualization
        qpos_list = []
        target_qpos_list = []
        all_actions = []
        t = 0

        while True:
            self.get_image()
            self.get_qpos()

            cur_joint_pos = np.array(self.qpos)
            img_np = self.image.astype(dtype=np.uint8)
            img_np = img_np.reshape((self.IMAGE_HEIGHT, self.IMAGE_WIDTH, 3))

            self.policy.eval()
            # 3. 处理并返回
            with torch.inference_mode():
                obs_image = img_np
                obs_qpos = cur_joint_pos[:self.STATE_DIM]

                qpos_numpy = np.array(obs_qpos)

                if self.is_degree:
                    qpos_numpy = qpos_numpy / np.pi * 180
                    qpos_numpy[-1] = obs_qpos[-1] / 0.04

                qpos = pre_process(qpos_numpy)
                qpos = torch.from_numpy(qpos).float().cuda().unsqueeze(0)
                qpos_history[:, t] = qpos
                curr_image = self.process_image(obs_image)
                curr_image = curr_image.unsqueeze(1)
                # print(curr_image.shape)
                # print(qpos.shape, curr_image.shape)
                ### query policy
                if self.config["policy_class"] == "ACT":
                    if t % query_frequency == 0:
                        all_actions = self.policy(qpos, curr_image)
                    if temporal_agg:
                        all_time_actions[[t], t : t + num_queries] = all_actions[:, :num_queries]
                        actions_for_curr_step = all_time_actions[:, t]
                        actions_populated = torch.all(actions_for_curr_step != 0, axis=1)
                        actions_for_curr_step = actions_for_curr_step[actions_populated]
                        exp_weights = np.exp(-self.exp_weight * np.arange(len(actions_for_curr_step)))
                        exp_weights = exp_weights / exp_weights.sum()
                        exp_weights = torch.from_numpy(exp_weights).cuda().unsqueeze(dim=1)
                        raw_action = (actions_for_curr_step * exp_weights).sum(dim=0, keepdim=True)
                    else:
                        raw_action = all_actions[:, t % query_frequency]
                elif self.config["policy_class"] == "CNNMLP":
                    raw_action = self.policy(qpos, curr_image)
                else:
                    raise NotImplementedError

                ### post-process actions
                raw_action = raw_action.squeeze(0).cpu().numpy()
                action = post_process(raw_action)
                target_qpos = action

                # TODO 仿真环境中机械臂采取target_qos
                t = t + 1

                ### for visualization
                qpos_list.append(qpos_numpy)
                target_qpos_list.append(target_qpos)

                if self.is_degree:
                    temp = target_qpos[-1]
                    target_qpos = target_qpos / 180 * np.pi
                    target_qpos[-1] = temp * 0.04

                self.pub_action(target_qpos)
