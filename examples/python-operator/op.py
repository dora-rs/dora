from typing import Callable
from enum import Enum

class DoraStatus(Enum):
    CONTINUE = 0
    STOP = 1

class Operator:
    """
    Example operator incrementing a counter every times its been called.

    The current value of the counter is sent back to dora on `counter`.
    """

    def __init__(self, counter=0):
        self.counter = counter

    def on_input(
        self,
        input_id: str,
        value: bytes,
        send_output: Callable[[str, bytes], None],
    ):
        """Handle input by incrementing count by one.

        Args:
            input_id (str): Id of the input declared in the yaml configuration
            value (bytes): Bytes message of the input
            send_output (Callable[[str, bytes]]): Function enabling sending output back to dora.
        """
        val_len = len(value)
        print(f"PYTHON received input {input_id}; value length: {val_len}")
        send_output("counter", (self.counter % 256).to_bytes(1, "little"))
        self.counter = self.counter + 1

        return DoraStatus.OK
