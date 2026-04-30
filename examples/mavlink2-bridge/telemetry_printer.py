"""Sink node for the mavlink2-bridge example.

Subscribes to the bridge's `heartbeat` output, decodes the per-row
StructArray fields, and prints a single line per heartbeat. Demonstrates
the MAVLink -> dora -> Python path of the bridge.
"""

import logging

from dora import Node


def _decode_heartbeat(struct_array) -> dict:
    """Pull out the first (and only) row of a HEARTBEAT StructArray."""
    rows = struct_array.to_pylist()
    if not rows:
        return {}
    return rows[0]


def main() -> None:
    node = Node()
    received = 0

    for event in node:
        if event["type"] == "INPUT":
            input_id = event["id"]
            if input_id == "heartbeat":
                row = _decode_heartbeat(event["value"])
                received += 1
                logging.info(
                    "heartbeat #%d custom_mode=%s mavtype=%s autopilot=%s system_status=%s",
                    received,
                    row.get("custom_mode"),
                    row.get("mavtype"),
                    row.get("autopilot"),
                    row.get("system_status"),
                )
            else:
                logging.warning("Unexpected input id: %s", input_id)
        elif event["type"] == "STOP":
            logging.info("telemetry-printer: STOP received after %d heartbeats", received)
            break

    logging.info("telemetry-printer: done (%d heartbeats decoded)", received)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    main()
