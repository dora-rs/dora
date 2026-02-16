"""TODO: Add docstring."""

from adora import AdoraStatus


class Operator:
    """Template docstring."""

    def __init__(self):
        """Perform initialization tasks."""

    def on_event(
        self,
        adora_event,
        send_output,
    ) -> AdoraStatus:
        """TODO :Description.

        Parameters
        ----------
        adora_event : dict
            Event containing an `id`, `data`, and `metadata`.
        send_output : Callable[[str, bytes | pa.Array, Optional[dict]], None]
            Function for sending output to the dataflow. The first argument is the `output_id`, the second
            argument is the data (either as bytes or a pa.Array), and the third argument is the adora metadata
            dictionary. For example:
                send_output("bbox", pa.array([100], type=pa.uint8()), adora_event["metadata"]).

        Returns
        -------
            AdoraStatus:
                CONTINUE means that the operator will
                    keep listening for further inputs.
                STOP means that the operator stop listening for inputs.

        """
        if adora_event["type"] == "INPUT":
            print(
                f"Received input {adora_event['id']}, with data: {adora_event['value']}",
            )

        return AdoraStatus.CONTINUE

    def __del__(self):
        """Perform actions before being deleted."""
