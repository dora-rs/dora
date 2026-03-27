"""Python receiver for cross-language test. Validates i64 Arrow arrays from Rust sender."""

import sys

from adora import Node


VALID_VALUES = {i * 10 for i in range(10)}  # {0, 10, 20, ..., 90}


def main():
    node = Node()
    received_count = 0

    for event in node:
        if event["type"] == "INPUT" and event["id"] == "values":
            values = event["value"].to_pylist()
            if len(values) != 1:
                print(f"python-receiver: ERROR expected 1 element, got {len(values)}")
                sys.exit(1)
            if values[0] not in VALID_VALUES:
                print(f"python-receiver: ERROR unexpected value {values[0]} (expected multiple of 10 in [0, 90])")
                sys.exit(1)
            print(f"python-receiver: validated value {values[0]}")
            received_count += 1
        elif event["type"] == "STOP":
            break

    if received_count < 5:
        print(f"python-receiver: ERROR got only {received_count} messages from Rust sender (expected >= 5)")
        sys.exit(1)

    print(f"python-receiver: SUCCESS - validated {received_count} messages")


if __name__ == "__main__":
    main()
