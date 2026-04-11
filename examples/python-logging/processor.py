"""Processor node demonstrating send_logs_as routing.

Receives sensor readings, computes a running average, and emits results.
Uses Python logging at various levels. Because the dataflow YAML sets
send_logs_as: log_entries, structured log entries from this node are
routed as data messages to any downstream node subscribing to
processor/log_entries.

Note: raw print() output is NOT routed by send_logs_as -- only parsed
structured log entries are forwarded.
"""

import json
import logging

import pyarrow as pa
from dora import Node


def main():
    node = Node()
    readings = []
    error_injected = False

    for event in node:
        if event["type"] == "INPUT":
            arr = event["value"]
            if len(arr) == 0:
                continue
            value = arr[0].as_py()
            readings.append(value)

            # Keep a sliding window of 20
            if len(readings) > 20:
                readings.pop(0)

            avg = sum(readings) / len(readings)
            logging.info("processed: avg=%.2f samples=%d", avg, len(readings))

            # Inject one error for demonstration
            if not error_injected and len(readings) >= 10:
                logging.error("simulated processing error at sample %d", len(readings))
                error_injected = True

            if avg > 26.0:
                logging.warning("average temperature elevated: %.2f", avg)

            # Raw print -- NOT captured by send_logs_as
            print(f"processor stdout: avg={avg:.2f}")

            result = pa.array([avg])
            node.send_output("result", result)

        elif event["type"] == "STOP":
            logging.info("processor stopping, processed %d readings", len(readings))
            break


if __name__ == "__main__":
    main()
