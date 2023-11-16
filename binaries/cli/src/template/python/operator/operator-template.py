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
        dora_event,
        send_output,
    ) -> DoraStatus:
        """

        Args:
            dora_event: Event containing an `id`, `data` and `metadata`.
            send_output Callable[[str, bytes | pa.Array, Optional[dict]], None]:
                Function for sending output to the dataflow:
                - First argument is the `output_id`
                - Second argument is the data as either bytes or `pa.Array`
                - Third argument is dora metadata dict
                e.g.: `send_output("bbox", pa.array([100], type=pa.uint8()), dora_event["metadata"])`

        Returns:
            DoraStatus:
                CONTINUE means that the operator will
                    keep listening for further inputs.
                STOP means that the operator stop listening for inputs.

        """
        if dora_event["type"] == "INPUT":
            print(
                f"Received input {dora_event['id']}, with data: {dora_event['value']}"
            )

        return DoraStatus.CONTINUE

    def __del__(self):
        """Called before being deleted"""
        pass
