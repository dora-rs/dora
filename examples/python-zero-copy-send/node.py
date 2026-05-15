#!/usr/bin/env python3
"""Zero-copy send example — driven by a `tick` input.

Demonstrates `node.send_output_raw(...)` which returns a `SampleHandler` that
exposes the pre-allocated output buffer via Python's buffer protocol. The
backing memory is allocated once by dora; the caller writes into it directly
(via memoryview or numpy) and `send()` ships it with no additional copies.

Two patterns are shown:

1. Context-manager form (recommended). `__exit__` calls `send()`.
2. Manual form. Useful when the send timing depends on logic that doesn't fit
   a `with` block. Caller must `release()` the memoryview before `send()`.

Requires Python >= 3.11 (the stable buffer-protocol slots).
"""

import numpy as np
from dora import Node


# Keep the example bounded so it works as a smoke test.
TICKS_TO_SEND = 10
LARGE_OUTPUT_BYTES = 64 * 1024  # 64 KiB — large enough to exercise SHM path
RGB_PIXEL_COUNT = 10_000
RGB_BYTES = RGB_PIXEL_COUNT * 3


def main() -> None:
    node = Node()
    ticks_seen = 0

    for event in node:
        if event["type"] == "STOP":
            break
        if event["type"] != "INPUT":
            continue

        ticks_seen += 1
        if ticks_seen > TICKS_TO_SEND:
            break

        # Pattern 1: context-manager form (recommended).
        # `with` returns a writable memoryview; `__exit__` calls `send()`.
        with node.send_output_raw("large_output", LARGE_OUTPUT_BYTES) as buf:
            arr = np.asarray(buf, dtype=np.uint8)
            arr[:] = np.random.randint(0, 256, size=LARGE_OUTPUT_BYTES, dtype=np.uint8)
            del arr  # release the numpy view before `with` block exits

        # Pattern 2: manual form. Useful when you need to make the send
        # conditional on logic that doesn't fit a `with` block.
        sample = node.send_output_raw(
            "rgb_image", RGB_BYTES, metadata={"width": 100, "height": 100}
        )
        mv = sample.as_memoryview()
        arr = np.asarray(mv, dtype=np.uint8).reshape(RGB_PIXEL_COUNT, 3)
        arr[:, 0] = ticks_seen  # R varies per tick
        arr[:, 1] = 64  # G
        arr[:, 2] = 128  # B
        del arr  # release derived numpy view
        mv.release()  # release the memoryview itself
        sample.send()

        print(f"tick {ticks_seen}/{TICKS_TO_SEND}: sent large_output + rgb_image")


if __name__ == "__main__":
    main()
