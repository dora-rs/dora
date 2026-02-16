"""TODO: Add docstring."""

import pyarrow as pa
from adora import AdoraStatus


class Operator:
    """Inferring object from images."""

    def __init__(self):
        """TODO: Add docstring."""
        self.last_file = ""
        self.last_path = ""
        self.last_netadata = None

    def on_event(
        self,
        adora_event,
        send_output,
    ) -> AdoraStatus:
        """TODO: Add docstring."""
        if adora_event["type"] == "INPUT" and adora_event["id"] == "file":
            input = adora_event["value"][0].as_py()

            with open(input["path"]) as file:
                self.last_file = file.read()
                self.last_path = input["path"]
                self.last_metadata = adora_event["metadata"]
            with open(input["path"], "w") as file:
                file.write(input["raw"])

            send_output(
                "saved_file",
                pa.array(
                    [
                        {
                            "raw": input["raw"],
                            "path": input["path"],
                            "origin": adora_event["id"],
                        },
                    ],
                ),
                adora_event["metadata"],
            )
        return AdoraStatus.CONTINUE
