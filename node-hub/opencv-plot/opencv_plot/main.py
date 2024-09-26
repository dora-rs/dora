import argparse
import os

import cv2
import numpy as np
import pyarrow as pa

from dora import Node

RUNNER_CI = True if os.getenv("CI") == "true" else False


class Plot:
    frame: np.array = np.array([])

    bboxes: dict = {
        "bbox": np.array([]),
        "conf": np.array([]),
        "labels": np.array([]),
    }

    text: str = ""

    width: np.uint32 = None
    height: np.uint32 = None


def plot_frame(plot):
    for bbox in zip(plot.bboxes["bbox"], plot.bboxes["conf"], plot.bboxes["labels"]):
        [
            [min_x, min_y, max_x, max_y],
            confidence,
            label,
        ] = bbox
        cv2.rectangle(
            plot.frame,
            (int(min_x), int(min_y)),
            (int(max_x), int(max_y)),
            (0, 255, 0),
            2,
        )

        cv2.putText(
            plot.frame,
            f"{label}, {confidence:0.2f}",
            (int(max_x) - 120, int(max_y) - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
            1,
        )

    cv2.putText(
        plot.frame,
        plot.text,
        (20, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 255, 255),
        1,
        1,
    )

    if plot.width is not None and plot.height is not None:
        plot.frame = cv2.resize(plot.frame, (plot.width, plot.height))

    if not RUNNER_CI:
        if len(plot.frame.shape) >= 3:
            cv2.imshow("Dora Node: opencv-plot", plot.frame)


def yuv420p_to_bgr_opencv(yuv_array, width, height):
    yuv = yuv_array.reshape((height * 3 // 2, width))
    return cv2.cvtColor(yuv, cv2.COLOR_YUV420p2RGB)


def main():

    # Handle dynamic nodes, ask for the name of the node in the dataflow, and the same values as the ENV variables.
    parser = argparse.ArgumentParser(
        description="OpenCV Plotter: This node is used to plot text and bounding boxes on an image."
    )

    parser.add_argument(
        "--name",
        type=str,
        required=False,
        help="The name of the node in the dataflow.",
        default="opencv-plot",
    )
    parser.add_argument(
        "--plot-width",
        type=int,
        required=False,
        help="The width of the plot.",
        default=None,
    )
    parser.add_argument(
        "--plot-height",
        type=int,
        required=False,
        help="The height of the plot.",
        default=None,
    )

    args = parser.parse_args()

    plot_width = os.getenv("PLOT_WIDTH", args.plot_width)
    plot_height = os.getenv("PLOT_HEIGHT", args.plot_height)

    if plot_width is not None:
        if isinstance(plot_width, str) and plot_width.isnumeric():
            plot_width = int(plot_width)

    if plot_height is not None:
        if isinstance(plot_height, str) and plot_height.isnumeric():
            plot_height = int(plot_height)

    node = Node(
        args.name
    )  # provide the name to connect to the dataflow if dynamic node
    plot = Plot()

    plot.width = plot_width
    plot.height = plot_height

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

                if encoding == "bgr8":
                    channels = 3
                    storage_type = np.uint8
                    plot.frame = (
                        storage.to_numpy()
                        .astype(storage_type)
                        .reshape((height, width, channels))
                        .copy()  # Copy So that we can add annotation on the image
                    )
                elif encoding == "rgb8":
                    channels = 3
                    storage_type = np.uint8
                    frame = (
                        storage.to_numpy()
                        .astype(storage_type)
                        .reshape((height, width, channels))
                    )

                    plot.frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

                elif encoding in ["jpeg", "jpg", "jpe", "bmp", "webp", "png"]:
                    channels = 3
                    storage_type = np.uint8
                    storage = storage.to_numpy()
                    plot.frame = cv2.imdecode(storage, cv2.IMREAD_COLOR)

                elif encoding == "yuv420":

                    storage = storage.to_numpy()

                    # Convert back to BGR results in more saturated image.
                    channels = 3
                    storage_type = np.uint8
                    img_bgr_restored = yuv420p_to_bgr_opencv(storage, width, height)

                    plot.frame = img_bgr_restored
                else:
                    raise RuntimeError(f"Unsupported image encoding: {encoding}")

                plot_frame(plot)
                if not RUNNER_CI:
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
            elif event_id == "bbox":
                arrow_bbox = event["value"][0]
                bbox_format = event["metadata"]["format"]

                if bbox_format == "xyxy":
                    bbox = arrow_bbox["bbox"].values.to_numpy().reshape(-1, 4)
                elif bbox_format == "xywh":
                    original_bbox = arrow_bbox["bbox"].values.to_numpy().reshape(-1, 4)
                    bbox = np.array(
                        [
                            (
                                x - w / 2,
                                y - h / 2,
                                x + w / 2,
                                y + h / 2,
                            )
                            for [x, y, w, h] in original_bbox
                        ]
                    )
                else:
                    raise RuntimeError(f"Unsupported bbox format: {bbox_format}")

                plot.bboxes = {
                    "bbox": bbox,
                    "conf": arrow_bbox["conf"].values.to_numpy(),
                    "labels": arrow_bbox["labels"].values.to_numpy(
                        zero_copy_only=False
                    ),
                }
            elif event_id == "text":
                plot.text = event["value"][0].as_py()
        elif event_type == "ERROR":
            raise RuntimeError(event["error"])


if __name__ == "__main__":
    main()
