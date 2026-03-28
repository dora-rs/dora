import pyarrow as pa
from dora import Node

def main():
    node = Node()
    counter = 0
    print("[requester-node] Starting up...")

    for event in node:
        if event["type"] == "INPUT" and event["id"] == "tick":
            counter += 1
            request_value = counter * 10
            print(f"[requester-node] Sending request #{counter} with value: {request_value}")

            reply = node.send_request("compute", pa.array([request_value], type=pa.int64()))
            reply_value = reply["value"][0].as_py()
            expected = request_value * 2

            if reply_value == expected:
                print(f"[requester-node] ✅ PASS — Reply {reply_value} matches expected {expected}")
            else:
                print(f"[requester-node] ❌ FAIL — Reply {reply_value} does NOT match expected {expected}")

            if counter >= 5:
                print(f"[requester-node] Completed {counter} round-trips. Stopping.")
                break

if __name__ == "__main__":
    main()