import cv2
import numpy as np
import pyarrow as pa
import torch
from dora import Node
from PIL import Image
from sam2.sam2_image_predictor import SAM2ImagePredictor

predictor = SAM2ImagePredictor.from_pretrained("facebook/sam2-hiera-large")


def main():
    pa.array([])  # initialize pyarrow array
    node = Node()
    frames = {}
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

            elif "boxes2d" in event_id:
                boxes2d = event["value"].to_numpy()
                metadata = event["metadata"]
                encoding = metadata["encoding"]
                if encoding != "xyxy":
                    raise RuntimeError(f"Unsupported boxes2d encoding: {encoding}")
                boxes2d = boxes2d.reshape(-1, 4)
                image_id = metadata["image_id"]
                with torch.inference_mode(), torch.autocast(
                    "cuda",
                    dtype=torch.bfloat16,
                ):
                    predictor.set_image(frames[image_id])
                    masks, _, _ = predictor.predict(box=boxes2d)
                    if len(masks.shape) == 4:
                        masks = masks[:, 0, :, :]
                    else:
                        masks = masks[0, :, :]

                    # masks = masks > 0
                    ## Mask to 3 channel image

                    node.send_output(
                        "masks",
                        pa.array(masks.ravel()),
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
