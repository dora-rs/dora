#!/usr/bin/env python3
"""
Example demonstrating zero-copy send using the Python node API.

This example shows how to use the `with ... as ...` syntax to allocate
a buffer, fill it with data, and send it efficiently without copying.
"""

import numpy as np
import pyarrow as pa
from dora import Node

def main():
    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            input_id = event["id"]

            if input_id == "tick":
                # Example 1: Zero-copy send with numpy
                data_size = 1024 * 1024  # 1MB

                # Allocate a sample using the context manager
                with node.send_output_raw("large_output", data_size) as sample:
                    # Get the writable numpy array wrapping the buffer (zero-copy)
                    np_array = sample.as_arrow()

                    # Fill the buffer with data
                    np_array[:] = np.random.randint(0, 256, size=data_size, dtype=np.uint8)

                    print(f"Prepared {data_size} bytes for zero-copy send")

                # The sample is automatically sent when exiting the `with` block
                print("Data sent!")

                # Example 2: Zero-copy send with struct-like data
                # Let's send a simple array of RGB pixels
                pixel_count = 100000
                rgb_size = pixel_count * 3  # 3 bytes per pixel (R, G, B)

                with node.send_output_raw("rgb_image", rgb_size, metadata={"width": 500, "height": 200}) as sample:
                    # Get the writable numpy array wrapping the buffer (zero-copy)
                    pixels = sample.as_arrow()

                    # Fill with gradient pattern
                    for i in range(pixel_count):
                        pixels[i*3] = (i % 256)      # Red
                        pixels[i*3 + 1] = ((i // 256) % 256)  # Green
                        pixels[i*3 + 2] = 128        # Blue

                    print(f"Prepared RGB image: {pixel_count} pixels")

                print("RGB image sent!")

        elif event["type"] == "STOP":
            print("Received stop signal")
            break

if __name__ == "__main__":
    main()
