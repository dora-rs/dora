#!/usr/bin/env python3
"""
Example demonstrating zero-copy send using the Python node API.

This example shows how to use the `with ... as ...` syntax to allocate
a buffer, fill it with data, and send it efficiently without copying.

The API now uses Python's built-in memoryview, so no external dependencies
are required. NumPy can optionally be used for convenience.
"""

from dora import Node

import numpy as np


def main():
    node = Node()

    # Example 1: Zero-copy send with memoryview (no dependencies)
    data_size = 1024 * 1024  # 1MB

    # Allocate a buffer using the context manager
    # The context manager returns a writable memoryview directly
    with node.send_output_raw("large_output", data_size) as buffer:
        np_array = np.asarray(buffer, copy=False)
        np_array[:] = np.random.randint(
            0, 256, size=data_size, dtype=np.uint8
        )
        print(f"Prepared {data_size} bytes for zero-copy send")


    # Example 2: Zero-copy send with struct-like data (RGB pixels)
    # This example works with pure Python memoryview - no dependencies!
    pixel_count = 100000
    rgb_size = pixel_count * 3  # 3 bytes per pixel (R, G, B)

    # Method 1: Manual send (commented out)
    for cnt in range(0, 10):
        sample = node.send_output_raw(
            "rgb_image", rgb_size, metadata={"width": 500, "height": 200}
        )
        mv = sample.as_memoryview()
        buffer = np.asarray(mv, np.uint8, copy=False)
        for i in range(pixel_count):
            buffer[i * 3] = cnt  # Red
            buffer[i * 3 + 1] = 10  # Green
            buffer[i * 3 + 2] = 10  # Blue
        sample.send()
        # Assignment after `send()` will trigger error
        # buffer[0] = 2  # This might raise undefined behavior

    # Method 2: Use `with ... as ...` (recommended)
    for cnt in range(0, 10):
        with node.send_output_raw(
            "rgb_image", rgb_size, metadata={"width": 500, "height": 200}
        ) as buffer:
            buffer = np.asarray(buffer, np.uint8, copy=False)
            # buffer is a writable memoryview - fill with solid color using pure Python
            for i in range(pixel_count):
                buffer[i * 3] = cnt  # Red
                buffer[i * 3 + 1] = 10  # Green
                buffer[i * 3 + 2] = 10  # Blue

        print(f"Prepared RGB image: {pixel_count} pixels")
    print("RGB image sent!")




if __name__ == "__main__":
    main()
