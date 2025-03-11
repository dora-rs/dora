"""TODO: Add docstring."""

import pyarrow as pa
from dora import DoraStatus


class Operator:
    """Inferring object from images."""

    def __init__(self):
        """TODO: Add docstring."""
        self.last_file = ""
        self.last_path = ""
        self.last_netadata = None

    def on_event(
        self,
        dora_event,
        send_output,
    ) -> DoraStatus:
        """TODO: Add docstring."""
        if dora_event["type"] == "INPUT" and dora_event["id"] == "file":
            input = dora_event["value"][0].as_py()

            with open(input["path"]) as file:
                self.last_file = file.read()
                self.last_path = input["path"]
                self.last_metadata = dora_event["metadata"]
            with open(input["path"], "w") as file:
                file.write(input["raw"])

            send_output(
                "saved_file",
                pa.array(
                    [
                        {
                            "raw": input["raw"],
                            "path": input["path"],
                            "origin": dora_event["id"],
                        },
                    ],
                ),
                dora_event["metadata"],
            )
        return DoraStatus.CONTINUE
