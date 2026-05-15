#!/usr/bin/env python3
"""Receiver for the zero-copy-send example.

Confirms each message arrives intact and prints a one-line summary per input.
Exits on STOP.
"""

from dora import Node


def main() -> None:
    node = Node()
    large_count = 0
    rgb_count = 0

    for event in node:
        if event["type"] == "STOP":
            print(
                f"STOP — received {large_count} large_output + {rgb_count} rgb_image"
            )
            break
        if event["type"] != "INPUT":
            continue

        input_id = event["id"]
        data = event["value"].to_numpy()

        if input_id == "large_output":
            large_count += 1
            print(
                f"[{large_count}] large_output: {len(data)} bytes, "
                f"min={data.min()} max={data.max()} mean={data.mean():.1f}"
            )

        elif input_id == "rgb_image":
            rgb_count += 1
            metadata = event.get("metadata", {})
            width = metadata.get("width")
            height = metadata.get("height")
            pixel_count = len(data) // 3
            r0, g0, b0 = data[0], data[1], data[2]
            print(
                f"[{rgb_count}] rgb_image: {pixel_count} pixels "
                f"({width}x{height}), first pixel RGB({r0}, {g0}, {b0})"
            )


if __name__ == "__main__":
    main()
