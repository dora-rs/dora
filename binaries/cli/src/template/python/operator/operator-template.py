"""TODO: Add docstring."""

from dora import DoraStatus


class Operator:
    """Template docstring."""

    def __init__(self):
        """Perform initialization tasks."""

    def on_event(
        self,
        dora_event,
        send_output,
    ) -> DoraStatus:
        """TODO :Description.

        Parameters
        ----------
        dora_event : dict
            Event containing an `id`, `data`, and `metadata`.
        send_output : Callable[[str, bytes | pa.Array, Optional[dict]], None]
            Function for sending output to the dataflow. The first argument is the `output_id`, the second
            argument is the data (either as bytes or a pa.Array), and the third argument is the dora metadata
            dictionary. For example:
                send_output("bbox", pa.array([100], type=pa.uint8()), dora_event["metadata"]).

        Returns
        -------
            DoraStatus:
                CONTINUE means that the operator will
                    keep listening for further inputs.
                STOP means that the operator stop listening for inputs.

        """
        if dora_event["type"] == "INPUT":
            print(
                f"Received input {dora_event['id']}, with data: {dora_event['value']}",
            )

        return DoraStatus.CONTINUE

    def __del__(self):
        """Perform actions before being deleted."""
