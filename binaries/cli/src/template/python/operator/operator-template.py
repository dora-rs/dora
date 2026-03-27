"""Python operator template for dora-rs.

Provides a skeleton for implementing custom dataflow operators in Python.
"""

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
        """Process events received from the dora-rs runtime.

        This method handles various event types, such as inputs from other nodes,
        and provides a mechanism to send outputs back to the dataflow.

        Args:
            dora_event (dict): The event received from dora-rs, containing
                fields like "type", "id", "value", and "metadata".
            send_output (Callable): A callback function to emit outputs.
                Args include output_id (str), value (bytes or pa.Array),
                and optional metadata (dict).

        Returns:
            DoraStatus: A status code (CONTINUE or STOP) that determines
                the operator's next action.

        """
        if dora_event["type"] == "INPUT":
            print(
                f"Received input {dora_event['id']}, with data: {dora_event['value']}",
            )

        return DoraStatus.CONTINUE

    def __del__(self):
        """Perform actions before being deleted."""
