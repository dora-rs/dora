"""Checker node: validates that each received array matches the expected value."""

from dora import Node

EXPECTED = [1, 2, 3, 4, 5]


def main():
    node = Node()
    count = 0
    for event in node:
        if event["type"] == "INPUT":
            received = event["value"].to_pylist()
            if received == EXPECTED:
                count += 1
                print(f"[PASS] #{count} data matches: {received}")
            else:
                print(f"[FAIL] expected {EXPECTED}, got {received}")
        elif event["type"] == "STOP":
            break
    print(f"Total PASS: {count}")


if __name__ == "__main__":
    main()
