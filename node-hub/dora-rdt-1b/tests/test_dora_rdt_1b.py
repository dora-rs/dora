import os

import numpy as np
import pytest
import torch
from PIL import Image
from torchvision import transforms

CI = os.environ.get("CI")

DEVICE = "cuda:0" if torch.cuda.is_available() else "cpu"
DEVICE = os.getenv("DEVICE", DEVICE)
DEVICE = torch.device(DEVICE)
DTYPE = torch.float16 if DEVICE != "cpu" else torch.float32


def test_import_main():
    # from dora_rdt_1b.main import main

    # Check that everything is working, and catch dora Runtime Exception as we're not running in a dora dataflow.
    # with pytest.raises(RuntimeError):
    # main()
    import dora_rdt_1b.RoboticsDiffusionTransformer as _  # noqa
    import dora_rdt_1b as _  # noqa


def test_download_policy():
    from dora_rdt_1b.main import get_policy

    ## in the future we should add this test within CI
    if CI:
        return
    rdt = get_policy()

    pytest.rdt = rdt


def test_download_vision_model():
    # Skip vision test as it is currently failing on macOS
    # See: https://github.com/dora-rs/dora/actions/runs/13484462433/job/37673857429
    pass
    from dora_rdt_1b.main import get_vision_model

    (vision_encoder, image_processor) = get_vision_model()
    pytest.vision_encoder = vision_encoder
    pytest.image_processor = image_processor


def test_download_language_embeddings():
    ## in the future we should add this test within CI
    if CI:
        return

    from dora_rdt_1b.main import get_language_embeddings

    pytest.lang_embeddings = get_language_embeddings()


def test_load_dummy_image():
    from dora_rdt_1b.main import config

    # Load pretrained model (in HF style)
    image_processor = pytest.image_processor
    vision_encoder = pytest.vision_encoder

    ## in the future we should add this test within CI
    if CI:
        return

    previous_image_path = "/path/to/img.jpeg"
    # previous_image = None # if t = 0
    previous_image = Image.open(previous_image_path).convert("RGB")  # if t > 0

    current_image_path = "/path/to/img.jpeg"
    current_image = Image.open(current_image_path).convert("RGB")

    # here I suppose you only have an image from exterior (e.g., 3rd person view) and you don't have any state information
    # the images should arrange in sequence [exterior_image, right_wrist_image, left_wrist_image] * image_history_size (e.g., 2)
    rgbs_lst = [[previous_image, None, None], [current_image, None, None]]
    # if your have an right_wrist_image, then it should be
    # rgbs_lst = [
    #     [previous_image, previous_right_wrist_image, None],
    #     [current_image, current_right_wrist_image, None]
    # ]

    # image pre-processing
    # The background image used for padding
    background_color = np.array(
        [int(x * 255) for x in image_processor.image_mean],
        dtype=np.uint8,
    ).reshape((1, 1, 3))
    background_image = (
        np.ones(
            (image_processor.size["height"], image_processor.size["width"], 3),
            dtype=np.uint8,
        )
        * background_color
    )

    image_tensor_list = []
    for step in range(config["common"]["img_history_size"]):
        rgbs = rgbs_lst[step % len(rgbs_lst)]
        for rgb in rgbs:
            if rgb is None:
                # Replace it with the background image
                image = Image.fromarray(background_image)
            else:
                image = rgb

            if config["dataset"].get("auto_adjust_image_brightness", False):
                pixel_values = list(image.getdata())
                average_brightness = sum(sum(pixel) for pixel in pixel_values) / (
                    len(pixel_values) * 255.0 * 3
                )
                if average_brightness <= 0.15:
                    image = transforms.ColorJitter(brightness=(1.75, 1.75))(image)

            if config["dataset"].get("image_aspect_ratio", "pad") == "pad":

                def expand2square(pil_img, background_color):
                    width, height = pil_img.size
                    if width == height:
                        return pil_img
                    if width > height:
                        result = Image.new(
                            pil_img.mode,
                            (width, width),
                            background_color,
                        )
                        result.paste(pil_img, (0, (width - height) // 2))
                        return result
                    result = Image.new(
                        pil_img.mode,
                        (height, height),
                        background_color,
                    )
                    result.paste(pil_img, ((height - width) // 2, 0))
                    return result

                image = expand2square(
                    image,
                    tuple(int(x * 255) for x in image_processor.image_mean),
                )
            image = image_processor.preprocess(image, return_tensors="pt")[
                "pixel_values"
            ][0]
            image_tensor_list.append(image)

    image_tensor = torch.stack(image_tensor_list, dim=0).to(DEVICE, dtype=DTYPE)
    # encode images
    image_embeds = vision_encoder(image_tensor).detach()
    pytest.image_embeds = image_embeds.reshape(
        -1,
        vision_encoder.hidden_size,
    ).unsqueeze(0)


def test_dummy_states():
    from dora_rdt_1b.main import config

    ## in the future we should add this test within CI
    if CI:
        return

    # suppose you do not have proprio
    # it's kind of tricky, I strongly suggest adding proprio as input and further fine-tuning
    B, N = 1, 1  # batch size and state history size
    states = torch.zeros(
        (B, N, config["model"]["state_token_dim"]),
        device=DEVICE,
        dtype=DTYPE,
    )

    # if you have proprio, you can do like this
    # format like this: [arm_joint_0_pos, arm_joint_1_pos, arm_joint_2_pos, arm_joint_3_pos, arm_joint_4_pos, arm_joint_5_pos, arm_joint_6_pos, gripper_open]
    # proprio = torch.tensor([0, 1, 2, 3, 4, 5, 6, 0.5]).reshape((1, 1, -1))
    # states[:, :, STATE_INDICES] = proprio

    state_elem_mask = torch.zeros(
        (B, config["model"]["state_token_dim"]),
        device=DEVICE,
        dtype=torch.bool,
    )
    from dora_rdt_1b.RoboticsDiffusionTransformer.configs.state_vec import (
        STATE_VEC_IDX_MAPPING,
    )

    # suppose you control in 7DOF joint position
    STATE_INDICES = [
        STATE_VEC_IDX_MAPPING["arm_joint_0_pos"],
        STATE_VEC_IDX_MAPPING["arm_joint_1_pos"],
        STATE_VEC_IDX_MAPPING["arm_joint_2_pos"],
        STATE_VEC_IDX_MAPPING["arm_joint_3_pos"],
        STATE_VEC_IDX_MAPPING["arm_joint_4_pos"],
        STATE_VEC_IDX_MAPPING["arm_joint_5_pos"],
        STATE_VEC_IDX_MAPPING["arm_joint_6_pos"],
        STATE_VEC_IDX_MAPPING["gripper_open"],
    ]

    state_elem_mask[:, STATE_INDICES] = True
    states, state_elem_mask = (
        states.to(DEVICE, dtype=DTYPE),
        state_elem_mask.to(DEVICE, dtype=DTYPE),
    )
    states = states[:, -1:, :]  # only use the last state
    pytest.states = states
    pytest.state_elem_mask = state_elem_mask
    pytest.STATE_INDICES = STATE_INDICES


def test_dummy_input():
    ## in the future we should add this test within CI
    if CI:
        return

    rdt = pytest.rdt
    lang_embeddings = pytest.lang_embeddings
    image_embeds = pytest.image_embeds
    state_elem_mask = pytest.state_elem_mask
    states = pytest.states
    STATE_INDICES = pytest.STATE_INDICES

    actions = rdt.predict_action(
        lang_tokens=lang_embeddings,
        lang_attn_mask=torch.ones(
            lang_embeddings.shape[:2],
            dtype=torch.bool,
            device=DEVICE,
        ),
        img_tokens=image_embeds,
        state_tokens=states,  # how can I get this?
        action_mask=state_elem_mask.unsqueeze(1),  # how can I get this?
        ctrl_freqs=torch.tensor([25.0], device=DEVICE),  # would this default work?
    )  # (1, chunk_size, 128)

    # select the meaning action via STATE_INDICES
    action = actions[
        :,
        :,
        STATE_INDICES,
    ]  # (1, chunk_size, len(STATE_INDICES)) = (1, chunk_size, 7+ 1)
    print(action)
