import pyarrow as pa
from dora import Node


def main():
    node = Node()
    requests_handled = 0
    print("[service-node] Starting up, waiting for requests...")

    for event in node:
        if event["type"] == "SERVICE_REQUEST":
            request_data = event["value"]
            request_value = request_data[0].as_py()
            requests_handled += 1

            reply_value = request_value * 2
            print(
                f"[service-node] Request #{requests_handled}: "
                f"received {request_value}, replying {reply_value}"
            )

            node.send_service_reply(
                event["id"],
                pa.array([reply_value], type=pa.int64()),
                event["metadata"],
            )
        elif event["type"] == "STOP":
            print(f"[service-node] Handled {requests_handled} requests total. Stopping.")
            break


if __name__ == "__main__":
    main()