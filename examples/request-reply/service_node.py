import pyarrow as pa
from dora import Node

def main():
    node = Node()
    print("[service-node] Starting up, waiting for requests...")

    for event in node:
        print(f"[service-node] Got event: type={event['type']}, id={event.get('id', 'N/A')}")

        if event["type"] == "SERVICE_REQUEST":
            request_data = event["value"]
            request_value = request_data[0].as_py()
            print(f"[service-node] Received request with value: {request_value}")

            reply_value = request_value * 2
            print(f"[service-node] Sending reply: {reply_value}")
            node.send_service_reply(
                event["id"],
                pa.array([reply_value], type=pa.int64()),
                event["metadata"],
            )

if __name__ == "__main__":
    main()