import pyarrow as pa
from dora import Node


def main():
    node = Node()
    ticks = 0
    all_passed = True
    phase = "data_types"
    print("[requester-node] Starting up...")

    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "request_tick":
                if phase == "data_types":
                    phase = "buffering"

                    print("[requester-node] --- Data type tests ---")
                    test_cases = [
                        ("int64", pa.array([10, 20, 30], type=pa.int64())),
                        ("float64", pa.array([1.5, 2.7, 3.14], type=pa.float64())),
                        ("uint8", pa.array([0, 127, 255], type=pa.uint8())),
                        ("string", pa.array(["hello", "world"])),
                        ("bool", pa.array([True, False, True])),
                        ("int32", pa.array([42], type=pa.int32())),
                        ("empty", pa.array([], type=pa.int64())),
                    ]

                    for name, data in test_cases:
                        try:
                            reply = node.send_request("compute", data, timeout=5.0)
                            if reply["value"] == data:
                                print(f"[requester-node] PASS -- {name}")
                            else:
                                print(f"[requester-node] FAIL -- {name} mismatch")
                                all_passed = False
                        except Exception as e:
                            print(f"[requester-node] FAIL -- {name}: {e}")
                            all_passed = False

                elif phase == "buffering":
                    phase = "done"

                    print(f"[requester-node] --- Buffering test ---")
                    print(f"[requester-node] Ticks buffered: {ticks}")
                    if ticks > 0:
                        print(f"[requester-node] PASS -- {ticks} ticks not lost")
                    else:
                        print(f"[requester-node] FAIL -- no ticks buffered")
                        all_passed = False

                    print(f"[requester-node] --- Timeout test ---")
                    try:
                        node.send_request(
                            "nonexistent_service",
                            pa.array([0], type=pa.int64()),
                            timeout=1.0,
                        )
                        print(f"[requester-node] FAIL -- should have timed out")
                        all_passed = False
                    except Exception:
                        print(f"[requester-node] PASS -- timed out as expected")

                    if all_passed:
                        print(f"[requester-node] ALL TESTS PASSED")
                    else:
                        print(f"[requester-node] SOME TESTS FAILED")
                    break

            elif event["id"] == "tick":
                ticks += 1


if __name__ == "__main__":
    main()