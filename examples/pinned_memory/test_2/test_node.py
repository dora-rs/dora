import time
import dora

class Operator:
    def __init__(self):
        self.node = None

    def on_event(
        self,
        event,
        send_output,
    ):
        event_type = event["type"]
        if event_type == "INPUT":
            input_id = event["id"]
            value = event.get("value")
            print(f"Received input {input_id}")
        elif event_type == "STOP":
            print("Received stop")
        elif event_type == "ERROR":
            print("Received error")
        else:
            print(f"Received unknown event {event_type}")

def init_operator(args):
    return Operator()

if __name__ == "__main__":
    dora.init(init_operator)