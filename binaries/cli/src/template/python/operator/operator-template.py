from typing import Callable

from enum import Enum


class DoraStatus(Enum):
    CONTINUE = 0
    STOP = 1


class Operator:
    """
    Template docstring
    """

    def __init__(self):
        """Called on initialisation"""
        pass

    def on_input(
        self,
        dora_input: dict,
        send_output: Callable[[str, bytes], None],
    ):
        """

        Args:
            dora_input (dict): Input dict containing an `id`, `data` and `metadata`.
            send_output (Callable[[str, bytes], None]): Send output to the dataflow

        Returns:
            DoraStatus:
                CONTINUE means that the operator will
                    keep listening for further inputs.
                STOP means that the operator stop listening for inputs.

        """
        return DoraStatus.CONTINUE

    def __del__(self):
        """Called before being deleted"""
        pass
