import time

import pyarrow as pa
from dora import Node

start = time.time()


def main() -> None:
    dora_node = Node()
    i = 0
    while time.time() - start < 10:
        dora_node.send_output("ts", pa.array([time.perf_counter_ns(), i]))
        i += 1
        # print(f"Sent {i} times", flush=True)
        time.sleep(0.001)
        if dora_node.next(timeout=0.001) is None:
            break


if __name__ == "__main__":
    main()
