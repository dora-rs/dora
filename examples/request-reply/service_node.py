import pyarrow as pa
from dora import Node


def main():
    node = Node()
    requests_handled = 0
    print("[service-node] Starting up, waiting for requests...")

    for event in node:
        if event["type"] == "SERVICE_REQUEST":
            requests_handled += 1
            print(f"[service-node] Request #{requests_handled}: type={event['value'].type}")

            node.send_service_reply(
                event["id"],
                event["value"],
                event["metadata"],
            )
        elif event["type"] == "STOP":
            print(f"[service-node] Handled {requests_handled} requests total. Stopping.")
            break


if __name__ == "__main__":
    main()