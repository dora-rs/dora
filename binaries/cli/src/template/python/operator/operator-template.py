from typing import Callable

from dora import DoraStatus


class Operator:
    """
    Template docstring
    """

    def __init__(self):
        """Called on initialisation"""
        pass

    def on_event(
        self,
        dora_event: dict,
        send_output: Callable[[str, bytes], None],
    ) -> DoraStatus:
        if dora_event["type"] == "INPUT":
            return self.on_input(dora_event, send_output)
        return DoraStatus.CONTINUE

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
        print(
            f"Received input {dora_input['id']}, with data: {dora_input['data']}"
        )

        return DoraStatus.CONTINUE

    def __del__(self):
        """Called before being deleted"""
        pass
