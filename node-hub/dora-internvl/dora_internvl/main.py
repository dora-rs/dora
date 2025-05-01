"""TODO: Add docstring."""

import os

import numpy as np
import pyarrow as pa
import torch
from dora import Node
from PIL import Image
from torchvision import transforms
from torchvision.transforms.functional import InterpolationMode
from transformers import AutoModel, AutoTokenizer

IMAGENET_MEAN = (0.485, 0.456, 0.406)
IMAGENET_STD = (0.229, 0.224, 0.225)


def build_transform(input_size):
    """TODO: Add docstring."""
    mean, std = IMAGENET_MEAN, IMAGENET_STD
    return transforms.Compose(
        [
            transforms.Lambda(lambda img: img.convert("RGB") if img.mode != "RGB" else img),
            transforms.Resize((input_size, input_size), interpolation=InterpolationMode.BICUBIC),
            transforms.ToTensor(),
            transforms.Normalize(mean=mean, std=std),
        ],
    )


def find_closest_aspect_ratio(aspect_ratio, target_ratios, width, height, image_size):
    """TODO: Add docstring."""
    best_ratio_diff = float("inf")
    best_ratio = (1, 1)
    area = width * height
    for ratio in target_ratios:
        target_aspect_ratio = ratio[0] / ratio[1]
        ratio_diff = abs(aspect_ratio - target_aspect_ratio)
        if ratio_diff < best_ratio_diff:
            best_ratio_diff = ratio_diff
            best_ratio = ratio
        elif ratio_diff == best_ratio_diff:
            if area > 0.5 * image_size * image_size * ratio[0] * ratio[1]:
                best_ratio = ratio
    return best_ratio


def dynamic_preprocess(
    image, min_num=1, max_num=12, image_size=448, use_thumbnail=False,
):
    """TODO: Add docstring."""
    orig_width, orig_height = image.size
    aspect_ratio = orig_width / orig_height

    # calculate the existing image aspect ratio
    target_ratios = set(
        (i, j)
        for n in range(min_num, max_num + 1)
        for i in range(1, n + 1)
        for j in range(1, n + 1)
        if i * j <= max_num and i * j >= min_num
    )
    target_ratios = sorted(target_ratios, key=lambda x: x[0] * x[1])

    # find the closest aspect ratio to the target
    target_aspect_ratio = find_closest_aspect_ratio(
        aspect_ratio, target_ratios, orig_width, orig_height, image_size,
    )

    # calculate the target width and height
    target_width = image_size * target_aspect_ratio[0]
    target_height = image_size * target_aspect_ratio[1]
    blocks = target_aspect_ratio[0] * target_aspect_ratio[1]

    # resize the image
    resized_img = image.resize((target_width, target_height))
    processed_images = []
    for i in range(blocks):
        box = (
            (i % (target_width // image_size)) * image_size,
            (i // (target_width // image_size)) * image_size,
            ((i % (target_width // image_size)) + 1) * image_size,
            ((i // (target_width // image_size)) + 1) * image_size,
        )
        # split the image
        split_img = resized_img.crop(box)
        processed_images.append(split_img)
    assert len(processed_images) == blocks
    if use_thumbnail and len(processed_images) != 1:
        thumbnail_img = image.resize((image_size, image_size))
        processed_images.append(thumbnail_img)
    return processed_images


def load_image(image_array: np.array, input_size=448, max_num=12):
    """TODO: Add docstring."""
    image = Image.fromarray(image_array).convert("RGB")
    transform = build_transform(input_size=input_size)
    images = dynamic_preprocess(
        image, image_size=input_size, use_thumbnail=True, max_num=max_num,
    )
    pixel_values = [transform(image) for image in images]
    return torch.stack(pixel_values)


def main():
    # Handle dynamic nodes, ask for the name of the node in the dataflow, and the same
    # values as the ENV variables.
    """TODO: Add docstring."""
    model_path = os.getenv("MODEL", "OpenGVLab/InternVL2-1B")
    device = "cuda:0" if torch.cuda.is_available() else "cpu"

    # If you want to load a model using multiple GPUs, please refer to the `Multiple GPUs` section.
    model = (
        AutoModel.from_pretrained(
            model_path,
            torch_dtype=torch.bfloat16,
            low_cpu_mem_usage=True,
            use_flash_attn=True,
            trust_remote_code=True,
        )
        .eval()
        .to(device)
    )
    tokenizer = AutoTokenizer.from_pretrained(
        model_path, trust_remote_code=True, use_fast=False,
    )

    node = Node()

    question = "<image>\nPlease describe the image shortly."
    frame = None
    pa.array([])  # initialize pyarrow array

    for event in node:
        event_type = event["type"]

        if event_type == "INPUT":
            event_id = event["id"]

            if event_id == "image":
                storage = event["value"]
                metadata = event["metadata"]
                encoding = metadata["encoding"]
                width = metadata["width"]
                height = metadata["height"]

                if encoding == "bgr8" or encoding == "rgb8":
                    channels = 3
                    storage_type = np.uint8
                else:
                    raise RuntimeError(f"Unsupported image encoding: {encoding}")

                frame = (
                    storage.to_numpy()
                    .astype(storage_type)
                    .reshape((height, width, channels))
                )
                if encoding == "bgr8":
                    frame = frame[:, :, ::-1]  # OpenCV image (BGR to RGB)
                elif encoding == "rgb8":
                    pass
                else:
                    raise RuntimeError(f"Unsupported image encoding: {encoding}")

            elif event_id == "text":
                question = "<image>\n" + event["value"][0].as_py()
                if frame is not None:
                    # set the max number of tiles in `max_num`
                    pixel_values = (
                        load_image(frame, max_num=12).to(torch.bfloat16).cuda()
                    )
                    generation_config = dict(max_new_tokens=1024, do_sample=True)
                    response = model.chat(
                        tokenizer, pixel_values, question, generation_config,
                    )
                    node.send_output(
                        "text",
                        pa.array([response]),
                        metadata,
                    )

        elif event_type == "ERROR":
            raise RuntimeError(event["error"])


if __name__ == "__main__":
    main()
