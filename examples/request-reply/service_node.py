"""
service_node.py — Simulates a "service" in the request-reply pattern.

Receives a numeric value in a request, computes a result (doubles it),
and sends back a reply. This node acts as the server side.
"""

from dora import Node


def main():
    node = Node()

    print("[service-node] Starting up, waiting for requests...")

    for event in node:
        if event["type"] == "INPUT":
            input_id = event["id"]

            if input_id == "request":
                # Extract the request value — expected to be a single integer
                request_data = event["value"]
                request_value = request_data[0].as_py()

                print(f"[service-node] Received request with value: {request_value}")

                # Compute the reply: double the input value
                reply_value = request_value * 2

                print(f"[service-node] Sending reply: {reply_value}")
                node.send_output(
                    output_id="reply",
                    data=pa.array([reply_value], type=pa.int64()),
                    metadata=event["metadata"],  # propagate metadata (e.g. timestamps)
                )
            else:
                print(f"[service-node] Ignoring unknown input: {input_id}")


if __name__ == "__main__":
    main()