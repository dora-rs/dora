# install dependencies as shown in the README here https://github.com/alik-git/RoboticsDiffusionTransformer?tab=readme-ov-file#installation
import yaml
import torch
import numpy as np
from PIL import Image
from torchvision import transforms

from dora_rdt_1b.RoboticsDiffusionTransformer.configs.state_vec import (
    STATE_VEC_IDX_MAPPING,
)
from dora_rdt_1b.RoboticsDiffusionTransformer.models.multimodal_encoder.siglip_encoder import (
    SiglipVisionTower,
)
from dora_rdt_1b.RoboticsDiffusionTransformer.models.rdt_runner import RDTRunner
from dora_rdt_1b.RoboticsDiffusionTransformer.configs.state_vec import (
    STATE_VEC_IDX_MAPPING,
)
from dora import Node
import cv2
import pyarrow as pa
import os
from pathlib import Path

VISION_DEFAULT_PATH = "robotics-diffusion-transformer/rdt-1b"
ROBOTIC_MODEL_NAME_OR_PATH = os.getenv(
    "ROBOTIC_MODEL_NAME_OR_PATH", VISION_DEFAULT_PATH
)
LANGUAGE_EMBEDDING_PATH = os.getenv("LANGUAGE_EMBEDDING", "lang_embed.pt")

VISION_DEFAULT_PATH = "google/siglip-so400m-patch14-384"
VISION_MODEL_NAME_OR_PATH = os.getenv("VISION_MODEL_NAME_OR_PATH", VISION_DEFAULT_PATH)


def get_policy():
    from dora_rdt_1b.RoboticsDiffusionTransformer.models.rdt_runner import RDTRunner

    pretrained_model_name_or_path = ROBOTIC_MODEL_NAME_OR_PATH
    rdt = RDTRunner.from_pretrained(pretrained_model_name_or_path)
    device = torch.device("cuda:0")
    dtype = torch.bfloat16  # recommanded
    rdt.to(device, dtype=dtype)
    rdt.eval()
    return rdt


def get_vision_model():
    from dora_rdt_1b.RoboticsDiffusionTransformer.models.multimodal_encoder.siglip_encoder import (
        SiglipVisionTower,
    )

    # Load vision encoder
    vision_encoder = SiglipVisionTower(
        vision_tower=VISION_MODEL_NAME_OR_PATH,
        args=None,
    )
    device = torch.device("cuda:0")
    dtype = torch.bfloat16  # recommanded
    vision_encoder.to(device, dtype=dtype)
    vision_encoder.eval()
    image_processor = vision_encoder.image_processor
    return vision_encoder, image_processor


def get_language_embeddings():
    device = torch.device("cuda:0")
    dtype = torch.bfloat16  # recommanded

    lang_embeddings = torch.load(
        LANGUAGE_EMBEDDING_PATH,
        map_location=device,
    )

    return lang_embeddings.unsqueeze(
        0
    )  # Size:  (B, L_lang, D) or None, language condition tokens (variable length),    dimension D is assumed to be the same as the hidden size.


def expand2square(pil_img, background_color):
    width, height = pil_img.size
    if width == height:
        return pil_img
    elif width > height:
        result = Image.new(pil_img.mode, (width, width), background_color)
        result.paste(pil_img, (0, (width - height) // 2))
        return result
    else:
        result = Image.new(pil_img.mode, (height, height), background_color)
        result.paste(pil_img, ((height - width) // 2, 0))
        return result


def process_image(rgbs_lst, image_processor, vision_encoder):
    device = torch.device("cuda:0")
    dtype = torch.bfloat16  # recommanded

    file_path = Path(__file__).parent

    config_path = (
        file_path / "RoboticsDiffusionTransformer/configs/base.yaml"
    )  # default config

    with open(config_path, "r") as fp:
        config = yaml.safe_load(fp)

    # previous_image_path = "/mnt/hpfs/1ms.ai/dora/node-hub/dora-rdt-1b/dora_rdt_1b/RoboticsDiffusionTransformer/img.jpeg"
    # # previous_image = None # if t = 0
    # previous_image = Image.fromarray(previous_image_path).convert("RGB")  # if t > 0

    # current_image_path = "/mnt/hpfs/1ms.ai/dora/node-hub/dora-rdt-1b/dora_rdt_1b/RoboticsDiffusionTransformer/img.jpeg"
    # current_image = Image.fromarray(current_image_path).convert("RGB")

    # here I suppose you only have an image from exterior (e.g., 3rd person view) and you don't have any state information
    # the images shoud arrange in sequence [exterior_image, right_wrist_image, left_wrist_image] * image_history_size (e.g., 2)
    # rgbs_lst = [[previous_image, None, None], [current_image, None, None]]
    # if your have an right_wrist_image, then it should be
    # rgbs_lst = [
    #     [previous_image, previous_right_wrist_image, None],
    #     [current_image, current_right_wrist_image, None]
    # ]

    # image pre-processing
    # The background image used for padding

    image_tensor_list = []
    for step in range(config["common"]["img_history_size"]):
        rgbs = rgbs_lst[step]
        for rgb in rgbs:
            assert rgb, "You should not have None image"
            image = rgb

            if config["dataset"].get("image_aspect_ratio", "pad") == "pad":
                background_color = tuple(
                    int(x * 255) for x in image_processor.image_mean
                )
                image = expand2square(image, background_color)
            image = image_processor.preprocess(image, return_tensors="pt")[
                "pixel_values"
            ][0]
            image_tensor_list.append(image)

    image_tensor = torch.stack(image_tensor_list, dim=0).to(device, dtype=dtype)
    # encode images
    image_embeds = vision_encoder(image_tensor).detach()
    return image_embeds.reshape(-1, vision_encoder.hidden_size).unsqueeze(0)


def get_states(proprio):
    device = torch.device("cuda:0")
    dtype = torch.bfloat16  # recommanded

    # suppose you control in 7DOF joint position
    STATE_INDICES = [
        STATE_VEC_IDX_MAPPING["left_arm_joint_0_pos"],
        STATE_VEC_IDX_MAPPING["left_arm_joint_1_pos"],
        STATE_VEC_IDX_MAPPING["left_arm_joint_2_pos"],
        STATE_VEC_IDX_MAPPING["left_arm_joint_3_pos"],
        STATE_VEC_IDX_MAPPING["left_arm_joint_4_pos"],
        STATE_VEC_IDX_MAPPING["left_arm_joint_5_pos"],
        STATE_VEC_IDX_MAPPING["left_gripper_open"],
        STATE_VEC_IDX_MAPPING["right_arm_joint_0_pos"],
        STATE_VEC_IDX_MAPPING["right_arm_joint_1_pos"],
        STATE_VEC_IDX_MAPPING["right_arm_joint_2_pos"],
        STATE_VEC_IDX_MAPPING["right_arm_joint_3_pos"],
        STATE_VEC_IDX_MAPPING["right_arm_joint_4_pos"],
        STATE_VEC_IDX_MAPPING["right_arm_joint_5_pos"],
        STATE_VEC_IDX_MAPPING["right_gripper_open"],
    ]

    file_path = Path(__file__).parent

    config_path = (
        file_path / "RoboticsDiffusionTransformer/configs/base.yaml"
    )  # default config
    with open(config_path, "r") as fp:
        config = yaml.safe_load(fp)

    B, N = 1, 1  # batch size and state history size
    states = torch.zeros(
        (B, N, config["model"]["state_token_dim"]), device=device, dtype=dtype
    )
    # suppose you do not have proprio
    # it's kind of tricky, I strongly suggest adding proprio as input and futher fine-tuning
    proprio = torch.tensor(proprio, device=device, dtype=dtype).reshape(
        (1, 1, -1)
    )  # B, N = 1, 1  # batch size and state history size

    # if you have proprio, you can do like this
    # format like this: [arm_joint_0_pos, arm_joint_1_pos, arm_joint_2_pos, arm_joint_3_pos, arm_joint_4_pos, arm_joint_5_pos, arm_joint_6_pos, gripper_open]
    # proprio = torch.tensor([0, 1, 2, 3, 4, 5, 6, 0.5]).reshape((1, 1, -1))
    states[:, :, STATE_INDICES] = proprio

    state_elem_mask = torch.zeros(
        (1, config["model"]["state_token_dim"]), device=device, dtype=torch.bool
    )

    state_elem_mask[:, STATE_INDICES] = True
    states, state_elem_mask = states.to(device, dtype=dtype), state_elem_mask.to(
        device, dtype=dtype
    )
    states = states[:, -1:, :]  # only use the last state
    return states, state_elem_mask, STATE_INDICES


def main():

    device = torch.device("cuda:0")
    rdt = get_policy()
    lang_embeddings = get_language_embeddings()
    vision_encoder, image_processor = get_vision_model()

    ## for image
    # image_embeds = process_image(rgb_lst, image_processor, vision_encoder)
    ## for states
    # states, state_elem_mask, STATE_INDICES = get_states(states)
    node = Node()
    frames = {}
    joints = {}
    with torch.no_grad():

        for event in node:
            event_type = event["type"]
            if event_type == "INPUT":

                event_id = event["id"]

                if "image" in event_id:
                    storage = event["value"]
                    metadata = event["metadata"]
                    encoding = metadata["encoding"]

                    if encoding == "bgr8":
                        channels = 3
                        storage_type = np.uint8
                    elif encoding == "rgb8":
                        channels = 3
                        storage_type = np.uint8
                    elif encoding in ["jpeg", "jpg", "jpe", "bmp", "webp", "png"]:
                        channels = 3
                        storage_type = np.uint8
                    else:
                        raise RuntimeError(f"Unsupported image encoding: {encoding}")

                    if encoding == "bgr8":
                        width = metadata["width"]
                        height = metadata["height"]
                        frame = (
                            storage.to_numpy()
                            .astype(storage_type)
                            .reshape((height, width, channels))
                        )
                        frame = frame[:, :, ::-1]  # OpenCV image (BGR to RGB)
                    elif encoding == "rgb8":
                        width = metadata["width"]
                        height = metadata["height"]
                        frame = (
                            storage.to_numpy()
                            .astype(storage_type)
                            .reshape((height, width, channels))
                        )
                    elif encoding in ["jpeg", "jpg", "jpe", "bmp", "webp", "png"]:
                        storage = storage.to_numpy()
                        frame = cv2.imdecode(storage, cv2.IMREAD_COLOR)
                        frame = frame[:, :, ::-1]  # OpenCV image (BGR to RGB)
                    else:
                        raise RuntimeError(f"Unsupported image encoding: {encoding}")
                    frames[f"last_{event_id}"] = frames.get(
                        event_id, Image.fromarray(frame)
                    )
                    frames[event_id] = Image.fromarray(frame)
                elif "jointstate" in event_id:
                    joints[event_id] = event["value"].to_numpy()

                elif "tick" == event_id:
                    ## Wait for all images
                    if len(frames.keys()) < 6:
                        continue
                    if len(joints.keys()) < 2:
                        continue

                    ## Embed images
                    rgbs_lst = [
                        [
                            frames["last_image_center"],
                            frames["last_image_right"],
                            frames["last_image_left"],
                        ],
                        [
                            frames["image_center"],
                            frames["image_right"],
                            frames["image_left"],
                        ],
                    ]
                    image_embeds = process_image(
                        rgbs_lst, image_processor, vision_encoder
                    )

                    ## Embed states
                    proprio = np.concatenate(
                        [
                            joints["jointstate_left"],
                            joints["jointstate_right"],
                        ]
                    )
                    states, state_elem_mask, state_indices = get_states(proprio=proprio)

                    actions = rdt.predict_action(
                        lang_tokens=lang_embeddings,
                        lang_attn_mask=torch.ones(
                            lang_embeddings.shape[:2], dtype=torch.bool, device=device
                        ),
                        img_tokens=image_embeds,
                        state_tokens=states,  # how can I get this?
                        action_mask=state_elem_mask.unsqueeze(1),  # how can I get this?
                        ctrl_freqs=torch.tensor(
                            [25.0], device=device
                        ),  # would this default work?
                    )  # (1, chunk_size, 128)

                    # select the meaning action via STATE_INDICES
                    action = actions[
                        :, :, state_indices
                    ]  # (1, chunk_size, len(STATE_INDICES)) = (1, chunk_size, 7+ 1)
                    action = action.detach().float().to("cpu").numpy()
                    node.send_output("action", pa.array(action.ravel()))
