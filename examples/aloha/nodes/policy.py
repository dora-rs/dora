from dora import DoraStatus


class Operator:
    def __init__(self):
        self.actions = ["get_food", "get_hat"]

    def on_event(self, event: dict, send_output) -> DoraStatus:
        if event["type"] == "INPUT":
            id = event["id"]
            # On initialization
            if id == "speech":
                text: str = event["value"][0].as_py().lower()
                # send_output("action", pa.array([""]))

        return DoraStatus.CONTINUE
