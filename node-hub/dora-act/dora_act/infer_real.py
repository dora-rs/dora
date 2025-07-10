import argparse
import csv
import os
import pickle
import sys
from datetime import datetime
from pathlib import Path
import time

import cv2
import numpy as np
import torch
from einops import rearrange

from constants import TASK_CONFIGS
from policy import ACTPolicy
from utils import set_seed

from dora import Node
import pyarrow as pa

from inference import ActInference

class ActInferenceReal(ActInference):
    def __init__(self):
        super().__init__()
        self.parser.add_argument("--num_rollout", action="store", type=int, default=1, required=False)
        args = vars(self.parser.parse_args())

        # command line parameters
        ckpt_dir = args["ckpt_dir"]
        policy_class = args["policy_class"]
        task_name = args["task_name"]
        num_epochs = args["num_epochs"]
        set_seed(args["seed"])

        # get task parameters
        task_config = TASK_CONFIGS[task_name]
        episode_len = task_config["episode_len"]
        camera_names = task_config["camera_names"]
        state_dim = task_config["state_dim"]

        # fixed parameters
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
        else:
            raise NotImplementedError

        self.config = {
            "num_epochs": num_epochs,
            "ckpt_dir": ckpt_dir,
            "episode_len": episode_len,
            "state_dim": state_dim,
            "policy_class": policy_class,
            "policy_config": policy_config,
            "task_name": task_name,
            "seed": args["seed"],
            "temporal_agg": args["temporal_agg"],
            "camera_names": camera_names,
            # "num_rollout": args["num_rollout"],
            "num_rollout": 1,
        }

        self.IMAGE_WIDTH = int(os.getenv("IMAGE_WIDTH", 640))
        self.IMAGE_HEIGHT = int(os.getenv("IMAGE_HEIGHT", 360))
    
    def make_policy(self, policy_class, policy_config):
        if policy_class == "ACT":
            policy = ACTPolicy(policy_config)
        else:
            raise NotImplementedError
        return policy
    
    def make_optimizer(self, policy_class, policy):
        if policy_class == "ACT":
            optimizer = policy.configure_optimizers()
        else:
            raise NotImplementedError
        return optimizer
    
    def get_image(self, request=True):
        super().get_image(request)
        self.image = self.image.reshape((self.IMAGE_HEIGHT, self.IMAGE_WIDTH, 3))
    
    def process_image(self):
        cur_images = []
        cur_image = rearrange(self.image, "h w c -> c h w")
        cur_images.append(cur_image)
        cur_image = np.stack(cur_images, axis=0)
        cur_image = torch.from_numpy(cur_image / 255.0).float().cuda().unsqueeze(0)
        return cur_image
    
    def inference(self):
        seed = self.config["seed"]
        ckpt_dir = self.config["ckpt_dir"]
        state_dim = self.config["state_dim"]
        policy_class = self.config["policy_class"]
        policy_config = self.config["policy_config"]
        camera_names = self.config["camera_names"]
        max_timesteps = self.config["episode_len"]
        temporal_agg = self.config["temporal_agg"]
        num_rollout = self.config["num_rollout"]
        task_name = self.config["task_name"]

        set_seed(seed)

        # load policy and stats
        ckpt_path = os.path.join(ckpt_dir, f"policy_best.ckpt")
        policy = self.make_policy(policy_class, policy_config)
        loading_status = policy.load_state_dict(torch.load(ckpt_path))
        print(loading_status)
        policy.cuda()
        policy.eval()
        print(f"Loaded: {ckpt_path}")
        stats_path = os.path.join(ckpt_dir, f"dataset_stats.pkl")
        with open(stats_path, "rb") as f:
            stats = pickle.load(f)

        # preprocess and postprocess
        pre_process = lambda s_qpos: (s_qpos - stats["qpos_mean"]) / stats["qpos_std"]
        post_process = lambda a: a * stats["action_std"] + stats["action_mean"]

        try:
            query_frequency = policy_config["num_queries"]  # num_queries == chunk_size
            if temporal_agg:  # temporal aggregation
                query_frequency = 1
                num_queries = policy_config["num_queries"]

            max_timesteps = int(max_timesteps * 3)  # may increase for real-world tasks

            image_history = []
            qpos_history = []
            target_qpos_history = []
            for rollout_id in range(num_rollout):
                # input(f"Rollout {rollout_id + 1}/{num_rollout} ready. Press Enter to start...")

                self.get_image()
                self.get_qpos()

                ### evaluation loop
                if temporal_agg:
                    all_time_actions = torch.zeros(
                        [max_timesteps, max_timesteps + num_queries, state_dim]
                    ).cuda()

                image_list = []
                qpos_list = []
                target_qpos_list = []
                with torch.inference_mode():
                    for t in range(max_timesteps):
                        ### process previous timestep to get qpos and image_list
                        qpos_numpy = np.array(self.qpos)
                        qpos = pre_process(qpos_numpy)
                        qpos = torch.from_numpy(qpos).float().cuda().unsqueeze(0)
                        curr_image = self.process_image()

                        image_list.append(self.image)
                        qpos_list.append(self.qpos)

                        ### query policy
                        if policy_class == "ACT":
                            if t % query_frequency == 0:
                                all_actions = policy(qpos, curr_image)
                            if temporal_agg:
                                all_time_actions[[t], t : t + num_queries] = all_actions
                                actions_for_curr_step = all_time_actions[:, t]
                                actions_populated = torch.all(actions_for_curr_step != 0, axis=1)
                                actions_for_curr_step = actions_for_curr_step[actions_populated]
                                k = 0.01
                                exp_weights = np.exp(-k * np.arange(len(actions_for_curr_step)))
                                exp_weights = exp_weights / exp_weights.sum()
                                exp_weights = torch.from_numpy(exp_weights).cuda().unsqueeze(dim=1)
                                raw_action = (actions_for_curr_step * exp_weights).sum(
                                    dim=0, keepdim=True
                                )
                            else:
                                raw_action = all_actions[:, t % query_frequency]
                        else:
                            raise NotImplementedError

                        ### post-process actions
                        raw_action = raw_action.squeeze(0).cpu().numpy()
                        action = post_process(raw_action)
                        target_qpos = action.tolist()

                        ### step the environment
                        self.pub_action(action=target_qpos)

                        self.get_image()
                        self.get_qpos()
                        
                        time.sleep(0.01)

                        target_qpos_list.append(target_qpos)

                print(f"Rollout {rollout_id + 1}/{num_rollout} finished")

                image_history.append(image_list)
                qpos_history.append(qpos_list)
                target_qpos_history.append(target_qpos_list)

        finally:
            # close environment
            print("Environment closed")

            # save images and qpos
            current_time = datetime.now().strftime("%Y_%m_%d_%H_%M")
            save_path = f"data/output/{task_name}/{current_time}"
            os.makedirs(save_path, exist_ok=True)
            for i in range(len(image_history)):
                images_path = os.path.join(save_path, f"image_list_{i}")
                os.makedirs(images_path, exist_ok=True)
                video_writer = cv2.VideoWriter(
                    os.path.join(save_path, f"video_{i}.mp4"),
                    cv2.VideoWriter_fourcc(*"mp4v"),
                    20,
                    (640, 360),
                )
                for j, image_np in enumerate(image_history[i]):
                    video_writer.write(image_np)
                    image_path = os.path.join(images_path, f"image_{j}.png")
                    cv2.imwrite(image_path, image_np)
                video_writer.release()

            for i in range(len(qpos_history)):
                qpos_path = os.path.join(save_path, f"qpos_{i}.csv")
                with open(qpos_path, "w", newline="") as file:
                    writer = csv.writer(file)
                    writer.writerow(
                        [
                            "joint_0",
                            "joint_1",
                            "joint_2",
                            "joint_3",
                            "joint_4",
                            "joint_5",
                            "joint_6",
                            "gripper width",
                        ]
                    )
                    for j in range(len(qpos_history[i])):
                        writer.writerow(qpos_history[i][j])

            for i in range(len(target_qpos_history)):
                target_qpos_path = os.path.join(save_path, f"target_qpos_{i}.csv")
                with open(target_qpos_path, "w", newline="") as file:
                    writer = csv.writer(file)
                    writer.writerow(
                        [
                            "joint_0",
                            "joint_1",
                            "joint_2",
                            "joint_3",
                            "joint_4",
                            "joint_5",
                            "joint_6",
                            "gripper width",
                        ]
                    )
                    for j in range(len(target_qpos_history[i])):
                        writer.writerow(target_qpos_history[i][j])

            print(f"Saved all images and qpos")
