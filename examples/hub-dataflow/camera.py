"""Generate synthetic RGB frames for the Dora Hub dataflow example."""

import numpy as np
import pyarrow as pa
from dora import Node

WIDTH = 320
HEIGHT = 240
BAR_WIDTH = WIDTH // 4
COLORS = ((255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 255))


def main():
    node = Node()
    frame_id = 0

    for event in node:
        if event["type"] == "STOP":
            break
        if event["type"] != "INPUT" or event["id"] != "tick":
            continue

        # Four vertical color bars form a small, changing RGB test pattern.
        row = b"".join(
            bytes(COLORS[(bar + frame_id) % len(COLORS)]) * BAR_WIDTH
            for bar in range(len(COLORS))
        )
        pixels = np.frombuffer(row * HEIGHT, dtype=np.uint8)
        node.send_output(
            "image",
            pa.array(pixels),
            metadata={
                "width": WIDTH,
                "height": HEIGHT,
                "encoding": "rgb8",
                "frame_id": frame_id,
            },
        )
        frame_id += 1


if __name__ == "__main__":
    main()
