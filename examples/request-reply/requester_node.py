import pyarrow as pa
from dora import Node


def main():
    node = Node()
    counter = 0
    ticks = 0
    total_rounds = 10
    all_passed = True
    print("[requester-node] Starting up...")

    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "request_tick":
                counter += 1
                request_value = counter * 10
                print(f"[requester-node] Sending request #{counter} with value: {request_value}")
                print(f"[requester-node] Ticks buffered so far: {ticks}")

                reply = node.send_request(
                    "compute",
                    pa.array([request_value], type=pa.int64()),
                    timeout=5.0,
                )
                reply_value = reply["value"][0].as_py()
                expected = request_value * 2

                if reply_value == expected:
                    print(
                        f"[requester-node] PASS -- Reply {reply_value} matches expected {expected}"
                    )
                else:
                    print(
                        f"[requester-node] FAIL -- Reply {reply_value} does NOT match expected {expected}"
                    )
                    all_passed = False

                if counter >= total_rounds:
                    print(f"[requester-node] --- Final results ---")
                    print(f"[requester-node] Round-trips completed: {counter}/{total_rounds}")
                    if ticks > 0:
                        print(
                            f"[requester-node] PASS -- {ticks} ticks buffered during blocking requests"
                        )
                    else:
                        print(
                            f"[requester-node] FAIL -- No ticks received, event buffering may be broken"
                        )
                        all_passed = False

                    # Test timeout: send a request with a short timeout
                    # and no server to reply (service already handled all
                    # requests but we send one more with a 1s timeout)
                    print(f"[requester-node] Testing timeout (1s)...")
                    try:
                        node.send_request(
                            "nonexistent_service",
                            pa.array([0], type=pa.int64()),
                            timeout=1.0,
                        )
                        print(f"[requester-node] FAIL -- Should have timed out")
                        all_passed = False
                    except Exception as e:
                        print(f"[requester-node] PASS -- Timed out as expected: {e}")

                    if all_passed:
                        print(f"[requester-node] ALL TESTS PASSED")
                    else:
                        print(f"[requester-node] SOME TESTS FAILED")
                    break

            elif event["id"] == "tick":
                ticks += 1


if __name__ == "__main__":
    main()