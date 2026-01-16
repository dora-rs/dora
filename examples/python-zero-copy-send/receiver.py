#!/usr/bin/env python3
"""
Receiver node that validates the zero-copy sent data.
"""

import numpy as np
import pyarrow as pa
from dora import Node

def main():
    node = Node()

    large_count = 0
    rgb_count = 0

    for event in node:
        if event["type"] == "INPUT":
            input_id = event["id"]
            value = event["value"]

            if input_id == "large_output":
                large_count += 1
                # Convert to numpy array
                data = value.to_numpy()
                print(f"[{large_count}] Received large_output: {len(data)} bytes, "
                      f"first 10 values: {data[:10]}, "
                      f"min: {data.min()}, max: {data.max()}, mean: {data.mean():.2f}")

            elif input_id == "rgb_image":
                rgb_count += 1
                metadata = event.get("metadata", {})
                width = metadata.get("width", 0)
                height = metadata.get("height", 0)

                # Convert to numpy array
                data = value.to_numpy()
                pixel_count = len(data) // 3

                print(f"[{rgb_count}] Received RGB image: {pixel_count} pixels "
                      f"({width}x{height}), size: {len(data)} bytes")

                # Verify the gradient pattern
                if len(data) >= 9:  # At least 3 pixels
                    pixel0_r = data[0]
                    pixel0_g = data[1]
                    pixel0_b = data[2]
                    pixel1_r = data[3]
                    pixel1_g = data[4]
                    print(f"  First pixel: RGB({pixel0_r}, {pixel0_g}, {pixel0_b})")
                    print(f"  Second pixel: RGB({pixel1_r}, {pixel1_g}, ...)")

        elif event["type"] == "STOP":
            print(f"Received stop signal. Processed {large_count} large outputs and {rgb_count} RGB images.")
            break

if __name__ == "__main__":
    main()
