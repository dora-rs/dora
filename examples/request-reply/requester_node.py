"""
requester_node.py — Simulates the "requester" in the request-reply pattern.

On each tick, sends a request value to the service node and then waits
for the corresponding reply. This node acts as the client side.

This example tests the request-reply feature by verifying that:
  1. A request is sent successfully as an output.
  2. The reply is received as an input and its value is correct.
  3. The round-trip works repeatedly without dropping messages.
"""

import pyarrow as pa
from dora import Node


def main():
    node = Node()

    counter = 0
    pending_request_value = None
    replies_received = 0

    print("[requester-node] Starting up...")

    for event in node:
        if event["type"] == "INPUT":
            input_id = event["id"]

            if input_id == "tick":
                # On each tick, send a new request
                counter += 1
                request_value = counter * 10  # e.g. 10, 20, 30, ...

                print(f"[requester-node] Sending request #{counter} with value: {request_value}")

                pending_request_value = request_value

                node.send_output(
                    output_id="request",
                    data=pa.array([request_value], type=pa.int64()),
                    metadata={},
                )

            elif input_id == "reply":
                # Receive the reply from the service node
                reply_data = event["value"]
                reply_value = reply_data[0].as_py()

                print(f"[requester-node] Received reply: {reply_value}")
                replies_received += 1

                # Verify correctness: reply should be exactly double the last sent request
                if pending_request_value is not None:
                    expected = pending_request_value * 2
                    if reply_value == expected:
                        print(
                            f"[requester-node] ✅ PASS — Reply {reply_value} matches expected {expected}"
                        )
                    else:
                        print(
                            f"[requester-node] ❌ FAIL — Reply {reply_value} does NOT match expected {expected}"
                        )
                    pending_request_value = None
                else:
                    print(
                        f"[requester-node] ⚠️  WARNING — Received unsolicited reply: {reply_value}"
                    )

                # Stop after 5 successful round-trips
                if replies_received >= 5:
                    print(
                        f"[requester-node] Completed {replies_received} request-reply round-trips. Stopping."
                    )
                    break

            else:
                print(f"[requester-node] Ignoring unknown input: {input_id}")


if __name__ == "__main__":
    main()