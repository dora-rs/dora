from dora import DoraStatus


class Operator:
    def __init__(self):
        self.requests_handled = 0

    def on_event(self, dora_event, send_output) -> DoraStatus:
        if dora_event["type"] == "SERVICE_REQUEST":
            self.requests_handled += 1
            print(
                f"[service-operator] Request #{self.requests_handled}: "
                f"type={dora_event['value'].type}"
            )
            dora_event["send_service_reply"](
                dora_event["id"],
                dora_event["value"],
                dora_event["metadata"],
            )
        elif dora_event["type"] == "STOP":
            print(
                f"[service-operator] Handled {self.requests_handled} requests. Stopping."
            )
        return DoraStatus.CONTINUE