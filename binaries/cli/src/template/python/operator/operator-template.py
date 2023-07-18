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
        send_output: Callable[[str, bytes, dict], None],
    ) -> DoraStatus:
        if dora_event["type"] == "INPUT":
            return self.on_input(dora_event, send_output)
        return DoraStatus.CONTINUE

    def on_input(
        self,
        dora_input: dict,
        send_output: Callable[[str, bytes, dict], None],
    ):
        """

        Args:
            dora_input (dict): Input dict containing an `id`, `data` and `metadata`.
            send_output Callable[[str, bytes | pa.UInt8Array, dict], None]: 
                Function for sending output to the dataflow:
                - First argument is the `output_id`
                - Second argument is the data as either bytes or `pa.UInt8Array` 
                - Third argument is dora metadata dict
                e.g.: `send_output("bbox", pa.array([100], type=pa.uint8()), dora_event["metadata"])`

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
