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
    text: {} = {
        "text": "",
        "font_scale": np.float32(0.0),
        "color": (np.uint8(0), np.uint8(0), np.uint8(0)),
        "thickness": np.uint32(0),
        "position": (np.uint32(0), np.uint32(0)),
    }

    width: np.uint32 = None
    height: np.uint32 = None


def plot_frame(plot, ci_enabled):
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
            2.0,
            (0, 255, 0),
            2,
            1,
        )
    cv2.putText(
        plot.frame,
        plot.text["text"],
        (int(plot.text["position"][0]), int(plot.text["position"][1])),
        cv2.FONT_HERSHEY_SIMPLEX,
        float(plot.text["font_scale"]),
        (int(plot.text["color"][0]), int(plot.text["color"][1]), int(plot.text["color"][2])),
        int(plot.text["thickness"]),
        1,
    )

    if plot.width is not None and plot.height is not None:
        plot.frame = cv2.resize(plot.frame, (plot.width, plot.height))

    if not ci_enabled:
        if len(plot.frame.shape) >= 3:
            cv2.imshow("Dora Node: opencv-plot", plot.frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                return True

    return False


def main():
    # Handle dynamic nodes, ask for the name of the node in the dataflow, and the width and height of the image.
    parser = argparse.ArgumentParser(
        description="OpenCV Plotter: This node is used to plot text and bounding boxes on an image.")

    parser.add_argument("--name", type=str, required=False, help="The name of the node in the dataflow.",
                        default="opencv-plot")
    parser.add_argument("--plot-width", type=int, required=False, help="The width of the plot.", default=None)
    parser.add_argument("--plot-height", type=int, required=False, help="The height of the plot.", default=None)

    args = parser.parse_args()

    plot_width = os.getenv("PLOT_WIDTH", args.plot_width)
    plot_height = os.getenv("PLOT_HEIGHT", args.plot_height)

    if plot_width is not None:
        if isinstance(plot_width, str) and plot_width.isnumeric():
            plot_width = int(plot_width)

    if plot_height is not None:
        if isinstance(plot_height, str) and plot_height.isnumeric():
            plot_height = int(plot_height)

    # check if the code is running in a CI environment (e.g. GitHub Actions) (parse to bool)
    ci_enabled = os.getenv("CI", False)
    if ci_enabled == "true":
        ci_enabled = True

    node = Node(args.name)  # provide the name to connect to the dataflow if dynamic node
    plot = Plot()

    plot.width = plot_width
    plot.height = plot_height

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
                arrow_image = event["value"][0]
                image = {
                    "width": np.uint32(arrow_image["width"].as_py()),
                    "height": np.uint32(arrow_image["height"].as_py()),
                    "channels": np.uint8(arrow_image["channels"].as_py()),
                    "data": arrow_image["data"].values.to_numpy().astype(np.uint8)
                }

                plot.frame = np.reshape(image["data"], (image["height"], image["width"], image["channels"]))

                if plot_frame(plot, ci_enabled):
                    break

            elif event_id == "bbox":
                bboxes = event["value"][0]["bbox"].values.to_numpy()
                conf = event["value"][0]["conf"].values.to_numpy()
                label = event["value"][0]["names"].values.to_pylist()

                plot.bboxes = np.reshape(bboxes, (-1, 4))
                plot.conf = conf
                plot.label = label

                if plot_frame(plot, ci_enabled):
                    break

            elif event_id == "text":
                arrow_text = event["value"][0]
                plot.text = {
                    "text": arrow_text["text"].as_py(),
                    "font_scale": np.float32(arrow_text["font_scale"].as_py()),
                    "color": (np.uint8(arrow_text["color"].as_py()[0]),
                              np.uint8(arrow_text["color"].as_py()[1]),
                              np.uint8(arrow_text["color"].as_py()[2])),
                    "thickness": np.uint32(arrow_text["thickness"].as_py()),
                    "position": (np.uint32(arrow_text["position"].as_py()[0]),
                                 np.uint32(arrow_text["position"].as_py()[1]))
                }

                if plot_frame(plot, ci_enabled):
                    break

        elif event_type == "STOP":
            break
        elif event_type == "ERROR":
            raise Exception(event["error"])


if __name__ == "__main__":
    main()
