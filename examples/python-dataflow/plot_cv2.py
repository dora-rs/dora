from enum import Enum
from typing import Callable

import cv2
import numpy as np


class DoraStatus(Enum):
    CONTINUE = 0
    STOP = 1


class Operator:
    """
    Example operator incrementing a counter every times its been called.

    The current value of the counter is sent back to dora on `counter`.
    """

    def __init__(self):
        self.counter = 0

    def on_input(
        self,
        input_id: str,
        value: bytes,
        send_output: Callable[[str, bytes], None],
    ) -> DoraStatus:
        """Handle input by incrementing count by one.

        Args:
            input_id (str): Id of the input declared in the yaml configuration
            value (bytes): Bytes message of the input
            send_output (Callable[[str, bytes]]): Function enabling sending output back to dora.
        """
        self.counter += 1
        if input_id == "image":
            frame = np.frombuffer(value, dtype="uint8")
            frame = np.reshape(frame, (480, 640, 3))
            cv2.imshow("frame", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                return DoraStatus.STOP
        if self.counter > 20:
            return DoraStatus.STOP
        else:
            return DoraStatus.CONTINUE

    def drop_operator(self):
        cv2.destroyAllWindows()
