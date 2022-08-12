from enum import Enum
from typing import Callable

import cv2


class DoraStatus(Enum):
    CONTINUE = 0
    STOP = 1


class Operator:
    """
    Example operator incrementing a counter every times its been called.

    The current value of the counter is sent back to dora on `counter`.
    """

    def __init__(self):
        self.video_capture = cv2.VideoCapture(0)
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

        ret, frame = self.video_capture.read()
        if ret:
            send_output("image", frame.tobytes())
        else:
            print("did not sent video")

        self.counter += 1
        if self.counter > 100:
            return DoraStatus.STOP

        return DoraStatus.CONTINUE

    def drop_operator(self):
        self.video_capture.release()
