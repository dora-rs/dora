"""Processor node that computes running average and logs warnings."""

import logging

import pyarrow as pa
from dora import Node


def main():
    node = Node()
    readings = []

    for event in node:
        if event["type"] == "INPUT":
            arr = event["value"]
            if len(arr) == 0:
                continue

            value = arr[0].as_py()
            readings.append(value)
            if len(readings) > 20:
                readings.pop(0)

            avg = sum(readings) / len(readings)
            logging.info("processor: avg=%.2f samples=%d", avg, len(readings))

            if avg > 26.0:
                logging.warning("elevated average temperature: %.2f C", avg)
            if avg > 28.0:
                logging.error("critical average temperature: %.2f C", avg)

            node.send_output("result", pa.array([avg]))

        elif event["type"] == "STOP":
            logging.info("processor stopping after %d readings", len(readings))
            break


if __name__ == "__main__":
    main()
