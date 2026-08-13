"""Sink node for the mavlink2-bridge example.

Subscribes to the bridge's `heartbeat` output, decodes the per-row
StructArray fields, and prints a single line per heartbeat. Demonstrates
the MAVLink -> dora -> Python path of the bridge.
"""

import logging
import time

from dora import Node

# Time budget for the FIRST heartbeat to arrive. Mirrors the Rust sink:
# the example's mavlink-sim emits a HEARTBEAT every 500 ms, so anything
# past a few seconds means the bridge connected but published nothing.
# The networked smoke harness has a 30 s budget and only signals STOP
# at cleanup, so the sink must fail fast on its own to flip the
# dataflow into Failed state before the harness times out.
FIRST_HEARTBEAT_DEADLINE_SECS = 8.0
# Polling cadence for `node.next(timeout=...)`; small enough that the
# warmup deadline above is honoured tightly.
RECV_POLL_SECS = 0.2


def _decode_heartbeat(struct_array) -> dict:
    """Pull out the first (and only) row of a HEARTBEAT StructArray."""
    rows = struct_array.to_pylist()
    if not rows:
        return {}
    return rows[0]


def main() -> None:
    node = Node()
    received = 0
    started = time.monotonic()

    while True:
        if received == 0 and (time.monotonic() - started) > FIRST_HEARTBEAT_DEADLINE_SECS:
            raise SystemExit(
                f"telemetry-printer: no HEARTBEAT received within "
                f"{FIRST_HEARTBEAT_DEADLINE_SECS:.0f}s; bridge appears silent — "
                f"smoke assertion failed"
            )
        event = node.next(timeout=RECV_POLL_SECS)
        if event is None:
            continue
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
    # Belt-and-suspenders for the local-mode smoke test, which uses
    # `dora run --stop-after`: STOP can arrive before the warmup
    # deadline above, so guard the post-loop case too.
    if received == 0:
        raise SystemExit(
            "telemetry-printer: bridge produced 0 heartbeats before STOP — "
            "smoke assertion failed (expected >=1 decoded HEARTBEAT)"
        )


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    main()
