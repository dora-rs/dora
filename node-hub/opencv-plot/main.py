import os
import argparse
import cv2

import numpy as np
import pyarrow as pa

from dora import Node


class Plot:
    frame: np.array = np.array([])
    bboxes: np.array = np.array([[]])
    conf: np.array = np.array([])
    label: np.array = np.array([])
    text: str = ""


def plot_frame(frame, plot, ci_enabled):
    for bbox in zip(plot.bboxes, plot.conf, plot.label):
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
            2,
            1,
        )

    width = frame.shape[1]
    height = frame.shape[0]

    # place plot.text at the bottom center of the frame
    cv2.putText(
        frame,
        plot.text,
        (int(width / 2) - 120, int(height) - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 255, 0),
        2,
        1,
    )

    if not ci_enabled:
        cv2.imshow("Dora Node: opencv-plot", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return True

    return False


def main():
    # Handle dynamic nodes, ask for the name of the node in the dataflow, and the width and height of the image.
    parser = argparse.ArgumentParser(
        description="OpenCV Plotter: This node is used to plot text and bounding boxes on an image.")

    parser.add_argument("--name", type=str, required=False, help="The name of the node in the dataflow.",
                        default="opencv-plot")
    parser.add_argument("--image-width", type=int, required=False, help="The width of the image.", default=640)
    parser.add_argument("--image-height", type=int, required=False, help="The height of the image.", default=480)

    args = parser.parse_args()

    image_width = os.getenv("IMAGE_WIDTH", args.image_width)
    image_height = os.getenv("IMAGE_HEIGHT", args.image_height)

    if image_width is not None:
        if isinstance(image_width, str) and image_width.isnumeric():
            image_width = int(image_width)

    if image_height is not None:
        if isinstance(image_height, str) and image_height.isnumeric():
            image_height = int(image_height)

    # check if the code is running in a CI environment (e.g. GitHub Actions) (parse to bool)
    ci_enabled = os.getenv("CI", False)
    if ci_enabled == "true":
        ci_enabled = True

    node = Node(args.name)  # provide the name to connect to the dataflow if dynamic node
    plot = Plot()

    pa.array([])  # initialize pyarrow array

    for event in node:
        event_type = event["type"]

        if event_type == "INPUT":
            event_id = event["id"]

            if event_id == "tick":
                node.send_output(
                    "tick",
                    pa.array([]),
                    event["metadata"]
                )

            elif event_id == "image":
                plot.frame = event["value"].to_numpy().reshape((image_height, image_width, 3)).copy()

                if plot_frame(plot.frame, plot, ci_enabled):
                    break

            elif event_id == "bbox":
                bboxes = event["value"][0]["bbox"].values.to_numpy()
                conf = event["value"][0]["conf"].values.to_numpy()
                label = event["value"][0]["names"].values.to_pylist()

                plot.bboxes = np.reshape(bboxes, (-1, 4))
                plot.conf = conf
                plot.label = label

                if plot_frame(plot.frame, plot, ci_enabled):
                    break

            elif event_id == "text":
                plot.text = event["value"][0].as_py()

                if plot_frame(plot.frame, plot, ci_enabled):
                    break

        elif event_type == "STOP":
            break
        elif event_type == "ERROR":
            raise Exception(event["error"])


if __name__ == "__main__":
    main()
