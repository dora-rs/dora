#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import numpy as np
import cv2
import pyarrow as pa
from pathlib import Path
from dora import Node
import subprocess

node = Node()

CAMERA_NAME = os.getenv("CAMERA_NAME", "camera")
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 800
FPS = 30

i = 0
episode = -1
dataflow_id = node.dataflow_id()

BASE = Path("out") / dataflow_id / "videos"
out_dir = BASE / f"{CAMERA_NAME}_episode_{episode:06d}"

for event in node:
    event_type = event["type"]
    if event_type == "INPUT":
        if event["id"] == "record_episode":
            record_episode = event["value"].to_numpy()[0]
            print(f"Recording episode {record_episode}", flush=True)
            # Save Episode Video
            if episode != -1 and record_episode == -1:
                out_dir = BASE / f"{CAMERA_NAME}_episode_{episode:06d}"
                fname = f"{CAMERA_NAME}_episode_{episode:06d}.mp4"
                video_path = BASE / fname
                # Save video
                ffmpeg_cmd = (
                    f"ffmpeg -r {FPS} "
                    "-f image2 "
                    "-loglevel error "
                    f"-i {str(out_dir / 'frame_%06d.png')} "
                    "-vcodec libx264 "
                    "-g 2 "
                    "-pix_fmt yuv444p "
                    f"{str(video_path)} &&"
                    f"rm -r {str(out_dir)}"
                )
                print(ffmpeg_cmd, flush=True)
                subprocess.Popen([ffmpeg_cmd], start_new_session=True, shell=True)
                episode = record_episode

            # Make new directory and start saving images
            elif episode == -1 and record_episode != -1:
                episode = record_episode
                out_dir = BASE / f"{CAMERA_NAME}_episode_{episode:06d}"
                out_dir.mkdir(parents=True, exist_ok=True)
                i = 0
            else:
                continue

        elif event["id"] == "image":
            # Only record image when in episode.
            # Episode 0 is for not recording periods.
            if episode == -1:
                continue

            fname = f"{CAMERA_NAME}_episode_{episode:06d}.mp4"
            node.send_output(
                "saved_image",
                pa.array([{"path": f"videos/{fname}", "timestamp": i / FPS}]),
                event["metadata"],
            )
            image = event["value"].to_numpy().reshape((CAMERA_HEIGHT, CAMERA_WIDTH, 3))
            path = str(out_dir / f"frame_{i:06d}.png")
            cv2.imwrite(path, image)
            i += 1
