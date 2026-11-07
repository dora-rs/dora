"""Generate synthetic RGB frames for the Dora Hub dataflow example."""

import pyarrow as pa
from dora import Node

WIDTH = 320
HEIGHT = 240
BAR_WIDTH = WIDTH // 4
COLORS = ((255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 255))

IMAGE_TYPE = pa.struct(
    [
        pa.field("width", pa.uint32()),
        pa.field("height", pa.uint32()),
        pa.field("encoding", pa.string()),
        pa.field("data", pa.large_binary()),
    ]
)


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
        image = pa.array(
            [
                {
                    "width": WIDTH,
                    "height": HEIGHT,
                    "encoding": "rgb8",
                    "data": row * HEIGHT,
                }
            ],
            type=IMAGE_TYPE,
        )
        node.send_output("image", image, metadata={"frame_id": frame_id})
        frame_id += 1


if __name__ == "__main__":
    main()
