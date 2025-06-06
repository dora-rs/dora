"""TODO: Add docstring."""

import cv2
import numpy as np
import pyarrow as pa
import torch
from dora import Node
from PIL import Image
from sam2.sam2_image_predictor import SAM2ImagePredictor

predictor = SAM2ImagePredictor.from_pretrained("facebook/sam2-hiera-large")


def main():
    """TODO: Add docstring."""
    pa.array([])  # initialize pyarrow array
    node = Node()
    frames = {}
    last_pred = None
    labels = None
    return_type = pa.Array
    image_id = None
    for event in node:
        event_type = event["type"]

        if event_type == "INPUT":
            event_id = event["id"]

            if "image" in event_id:
                storage = event["value"]
                metadata = event["metadata"]
                encoding = metadata["encoding"]
                width = metadata["width"]
                height = metadata["height"]

                if (
                    encoding == "bgr8"
                    or encoding == "rgb8"
                    or encoding in ["jpeg", "jpg", "jpe", "bmp", "webp", "png"]
                ):
                    channels = 3
                    storage_type = np.uint8
                else:
                    error = f"Unsupported image encoding: {encoding}"
                    raise RuntimeError(error)

                if encoding == "bgr8":
                    frame = (
                        storage.to_numpy()
                        .astype(storage_type)
                        .reshape((height, width, channels))
                    )
                    frame = frame[:, :, ::-1]  # OpenCV image (BGR to RGB)
                elif encoding == "rgb8":
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
                image = Image.fromarray(frame)
                frames[event_id] = image

                # TODO: Fix the tracking code for SAM2.
                continue
                if last_pred is not None:
                    with (
                        torch.inference_mode(),
                        torch.autocast(
                            "cuda",
                            dtype=torch.bfloat16,
                        ),
                    ):
                        predictor.set_image(frames[image_id])

                        new_logits = []
                        new_masks = []

                        if len(last_pred.shape) < 3:
                            last_pred = np.expand_dims(last_pred, 0)

                        for mask in last_pred:
                            mask = np.expand_dims(mask, 0)  # Make shape: 1x256x256
                            masks, _, new_logit = predictor.predict(
                                mask_input=mask,
                                multimask_output=False,
                            )
                            if len(masks.shape) == 4:
                                masks = masks[:, 0, :, :]
                            else:
                                masks = masks[0, :, :]

                            masks = masks > 0
                            new_masks.append(masks)
                            new_logits.append(new_logit)
                            ## Mask to 3 channel image

                        last_pred = np.concatenate(new_logits, axis=0)
                        masks = np.concatenate(new_masks, axis=0)

                        match return_type:
                            case pa.Array:
                                node.send_output(
                                    "masks",
                                    pa.array(masks.ravel()),
                                    metadata={
                                        "image_id": image_id,
                                        "width": frames[image_id].width,
                                        "height": frames[image_id].height,
                                    },
                                )
                            case pa.StructArray:
                                node.send_output(
                                    "masks",
                                    pa.array(
                                        [
                                            {
                                                "masks": masks.ravel(),
                                                "labels": event["value"]["labels"],
                                            },
                                        ],
                                    ),
                                    metadata={
                                        "image_id": image_id,
                                        "width": frames[image_id].width,
                                        "height": frames[image_id].height,
                                    },
                                )

            if "boxes2d" in event_id:
                if len(event["value"]) == 0:
                    node.send_output("masks", pa.array([]))
                    continue
                if isinstance(event["value"], pa.StructArray):
                    boxes2d = event["value"][0].get("bbox").values.to_numpy()
                    labels = (
                        event["value"][0]
                        .get("labels")
                        .values.to_numpy(zero_copy_only=False)
                    )
                    return_type = pa.Array
                else:
                    boxes2d = event["value"].to_numpy()
                    labels = None
                    return_type = pa.Array

                metadata = event["metadata"]
                encoding = metadata["encoding"]
                if encoding != "xyxy":
                    raise RuntimeError(f"Unsupported boxes2d encoding: {encoding}")
                boxes2d = boxes2d.reshape(-1, 4)
                image_id = metadata["image_id"]
                with (
                    torch.inference_mode(),
                    torch.autocast(
                        "cuda",
                        dtype=torch.bfloat16,
                    ),
                ):
                    predictor.set_image(frames[image_id])
                    masks, _scores, last_pred = predictor.predict(
                        box=boxes2d,
                        point_labels=labels,
                        multimask_output=False,
                    )

                    if len(masks.shape) == 4:
                        masks = masks[:, 0, :, :]
                        last_pred = last_pred[:, 0, :, :]
                    else:
                        masks = masks[0, :, :]
                        last_pred = last_pred[0, :, :]

                    masks = masks > 0
                    metadata["image_id"] = image_id
                    metadata["width"] = frames[image_id].width
                    metadata["height"] = frames[image_id].height
                    ## Mask to 3 channel image
                    match return_type:
                        case pa.Array:
                            node.send_output("masks", pa.array(masks.ravel()), metadata)
                        case pa.StructArray:
                            node.send_output(
                                "masks",
                                pa.array(
                                    [
                                        {
                                            "masks": masks.ravel(),
                                            "labels": event["value"]["labels"],
                                        },
                                    ],
                                ),
                                metadata,
                            )
            elif "points" in event_id:
                points = event["value"].to_numpy().reshape((-1, 2))
                return_type = pa.Array
                if len(frames) == 0:
                    continue
                first_image = next(iter(frames.keys()))
                image_id = event["metadata"].get("image_id", first_image)
                with (
                    torch.inference_mode(),
                    torch.autocast(
                        "cuda",
                        dtype=torch.bfloat16,
                    ),
                ):
                    predictor.set_image(frames[image_id])
                    labels = [i for i in range(len(points))]
                    masks, _scores, last_pred = predictor.predict(
                        points,
                        point_labels=labels,
                        multimask_output=False,
                    )

                    if len(masks.shape) == 4:
                        masks = masks[:, 0, :, :]
                        last_pred = last_pred[:, 0, :, :]
                    else:
                        masks = masks[0, :, :]
                        last_pred = last_pred[0, :, :]

                    masks = masks > 0
                    ## Mask to 3 channel image
                    match return_type:
                        case pa.Array:
                            node.send_output(
                                "masks",
                                pa.array(masks.ravel()),
                                metadata={
                                    "image_id": image_id,
                                    "width": frames[image_id].width,
                                    "height": frames[image_id].height,
                                },
                            )
                        case pa.StructArray:
                            node.send_output(
                                "masks",
                                pa.array(
                                    [
                                        {
                                            "masks": masks.ravel(),
                                            "labels": event["value"]["labels"],
                                        },
                                    ],
                                ),
                                metadata={
                                    "image_id": image_id,
                                    "width": frames[image_id].width,
                                    "height": frames[image_id].height,
                                },
                            )

        elif event_type == "ERROR":
            print("Event Error:" + event["error"])


if __name__ == "__main__":
    main()
