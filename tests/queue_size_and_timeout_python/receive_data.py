import time

from dora import Node


def main() -> None:
    dora_node = Node()

    i = 0
    while True:
        message = dora_node.next(timeout=0.05)
        if message is None:
            break

        if message["type"] != "INPUT":
            continue
        sent = message["value"][0].as_py()
        j = message["value"][1].as_py()
        sent_in_s = sent / 1_000_000_000
        received = time.perf_counter_ns()
        received_in_s = received / 1_000_000_000

        i += 1
        print(
            f"[{i}, {j}] Sent: {sent_in_s}, Received: {received_in_s}, Difference: {received_in_s - sent_in_s}"
        )
        assert received_in_s - sent_in_s < 1.0
        time.sleep(0.1)


if __name__ == "__main__":
    main()
