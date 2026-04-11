"""Noisy sensor node demonstrating log level filtering and rotation.

Emits a mix of print() output, logging.info(), logging.debug(), and
logging.warning() messages. Combined with min_log_level: info in the
dataflow YAML, debug messages are suppressed at the daemon before they
reach log files or the terminal.

max_log_size: "1KB" triggers log rotation quickly so you can observe
rotated files in out/<dataflow_uuid>/.
"""

import logging
import random

import pyarrow as pa
from dora import Node


def main():
    node = Node()
    # Note: do NOT call logging.basicConfig() before Node().
    # Node() installs a tracing bridge that routes Python logging
    # through Rust tracing for structured log capture.
    count = 0

    for event in node:
        if event["type"] == "INPUT":
            count += 1
            value = random.uniform(18.0, 30.0)

            # Raw stdout -- captured as "stdout" level
            print(f"raw sensor tick {count}")

            # Structured logs at various levels
            logging.debug("sensor detail: tick=%d raw=%.4f", count, value)
            logging.info("sensor reading: tick=%d value=%.2f", count, value)

            if value > 28.0:
                logging.warning("sensor high temp: %.2f at tick %d", value, count)

            node.send_output("reading", pa.array([value]))

        elif event["type"] == "STOP":
            logging.info("sensor stopping after %d ticks", count)
            break


if __name__ == "__main__":
    main()
