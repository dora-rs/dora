"""Closed-loop SITL mission driven from a dora node.

State machine:

    WAIT_HEARTBEAT -> SET_GUIDED -> ARM -> TAKEOFF -> HOVER ->
    LAND -> DISARM -> DONE

Each command-issuing state sends a single COMMAND_LONG to the bridge
on output ``command_long_cmd`` and waits for a matching COMMAND_ACK
on input ``command_ack``. Position-progress states (TAKEOFF, LAND)
also watch ``global_position_int`` for relative_alt to cross a
threshold. Per-state timeouts bound the worst case so a hung step
fails loudly instead of running forever.

This example is local-only. It does not run in CI -- ArduPilot SITL
needs to be installed and started manually before the dataflow boots
(see ``scripts/start_sitl.sh``).
"""

from __future__ import annotations

import logging
import os
import time
from dataclasses import dataclass

import pyarrow as pa
from dora import Node

# MAV_CMD enum values (mavlink common dialect).
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_NAV_LAND = 21
MAV_CMD_DO_SET_MODE = 176
MAV_CMD_COMPONENT_ARM_DISARM = 400

# MAV_RESULT enum values.
MAV_RESULT_ACCEPTED = 0
MAV_RESULT_NAMES = {
    0: "ACCEPTED",
    1: "TEMPORARILY_REJECTED",
    2: "DENIED",
    3: "UNSUPPORTED",
    4: "FAILED",
    5: "IN_PROGRESS",
    6: "CANCELLED",
}

# MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1; ArduCopter custom_mode for GUIDED is 4.
ARDUCOPTER_GUIDED_CUSTOM_MODE = 4
MODE_FLAG_CUSTOM_ENABLED = 1.0

# Tunables.
TARGET_SYSTEM = int(os.environ.get("MAVLINK_SITL_TARGET_SYSTEM", "1"))
TARGET_COMPONENT = int(os.environ.get("MAVLINK_SITL_TARGET_COMPONENT", "1"))
TAKEOFF_ALT_M = float(os.environ.get("MAVLINK_SITL_TAKEOFF_ALT_M", "10.0"))
HOVER_SECS = float(os.environ.get("MAVLINK_SITL_HOVER_SECS", "5.0"))
COMMAND_TIMEOUT_SECS = float(os.environ.get("MAVLINK_SITL_COMMAND_TIMEOUT", "15.0"))
ALT_TIMEOUT_SECS = float(os.environ.get("MAVLINK_SITL_ALT_TIMEOUT", "30.0"))
ALT_REACHED_M = 0.85 * TAKEOFF_ALT_M  # accept if within 15% of target
LAND_DETECT_M = 0.5  # consider landed when relative_alt < 0.5 m


@dataclass
class Pending:
    """A command we sent and are waiting on an ACK for."""

    command: int
    description: str
    sent_at: float


def _make_command_long(
    command: int,
    p1: float = 0.0,
    p2: float = 0.0,
    p3: float = 0.0,
    p4: float = 0.0,
    p5: float = 0.0,
    p6: float = 0.0,
    p7: float = 0.0,
) -> pa.StructArray:
    """Build a 1-row StructArray matching the bridge's COMMAND_LONG schema.

    Field names + types must match
    ``libraries/extensions/mavlink2-bridge/src/arrow_convert.rs``:
    params are float32, command is uint32, target_* and confirmation
    are uint8.
    """
    return pa.StructArray.from_arrays(
        [
            pa.array([p1], type=pa.float32()),
            pa.array([p2], type=pa.float32()),
            pa.array([p3], type=pa.float32()),
            pa.array([p4], type=pa.float32()),
            pa.array([p5], type=pa.float32()),
            pa.array([p6], type=pa.float32()),
            pa.array([p7], type=pa.float32()),
            pa.array([command], type=pa.uint32()),
            pa.array([TARGET_SYSTEM], type=pa.uint8()),
            pa.array([TARGET_COMPONENT], type=pa.uint8()),
            pa.array([0], type=pa.uint8()),  # confirmation
        ],
        names=[
            "param1",
            "param2",
            "param3",
            "param4",
            "param5",
            "param6",
            "param7",
            "command",
            "target_system",
            "target_component",
            "confirmation",
        ],
    )


def _first_row(struct_array: pa.StructArray) -> dict:
    rows = struct_array.to_pylist()
    return rows[0] if rows else {}


def _relative_alt_m(global_position_int: pa.StructArray) -> float | None:
    row = _first_row(global_position_int)
    relative_alt_mm = row.get("relative_alt")
    if relative_alt_mm is None:
        return None
    return relative_alt_mm / 1000.0


class Mission:
    def __init__(self, node: Node) -> None:
        self.node = node
        self.pending: Pending | None = None
        self.last_relative_alt_m: float | None = None

    # ---- low-level helpers -------------------------------------------------

    def _send(self, description: str, command: int, **params: float) -> None:
        cmd = _make_command_long(command, **params)
        self.node.send_output("command_long_cmd", cmd)
        self.pending = Pending(command=command, description=description, sent_at=time.monotonic())
        logging.info("mission: -> %s (cmd=%d)", description, command)

    def _await_ack(self, timeout: float) -> None:
        """Block until COMMAND_ACK for self.pending arrives, or fail."""
        assert self.pending is not None
        deadline = time.monotonic() + timeout
        for event in self.node:
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"no ACK for {self.pending.description} within {timeout:.0f}s"
                )
            if event["type"] != "INPUT":
                if event["type"] == "STOP":
                    raise RuntimeError("STOP received mid-mission")
                continue
            input_id = event["id"]
            if input_id == "global_position_int":
                self.last_relative_alt_m = _relative_alt_m(event["value"])
                continue
            if input_id == "command_ack":
                row = _first_row(event["value"])
                if row.get("command") != self.pending.command:
                    logging.debug("mission: ignoring ACK for cmd=%s", row.get("command"))
                    continue
                result = row.get("result", -1)
                name = MAV_RESULT_NAMES.get(result, f"UNKNOWN({result})")
                if result != MAV_RESULT_ACCEPTED:
                    raise RuntimeError(
                        f"{self.pending.description}: ACK = {name} (result={result})"
                    )
                logging.info(
                    "mission: <- ACK %s [%s, %.1fs]",
                    self.pending.description,
                    name,
                    time.monotonic() - self.pending.sent_at,
                )
                self.pending = None
                return
        raise RuntimeError(f"event stream ended before ACK for {self.pending.description}")

    def _await_alt_at_least(self, target_m: float, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        for event in self.node:
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"altitude did not reach {target_m:.1f} m within {timeout:.0f}s "
                    f"(last relative_alt={self.last_relative_alt_m})"
                )
            if event["type"] == "STOP":
                raise RuntimeError("STOP received mid-mission")
            if event["type"] != "INPUT" or event["id"] != "global_position_int":
                continue
            alt = _relative_alt_m(event["value"])
            if alt is None:
                continue
            self.last_relative_alt_m = alt
            if alt >= target_m:
                logging.info("mission: reached %.2f m (target %.2f)", alt, target_m)
                return

    def _await_alt_below(self, target_m: float, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        for event in self.node:
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"altitude did not drop below {target_m:.1f} m within {timeout:.0f}s "
                    f"(last relative_alt={self.last_relative_alt_m})"
                )
            if event["type"] == "STOP":
                raise RuntimeError("STOP received mid-mission")
            if event["type"] != "INPUT" or event["id"] != "global_position_int":
                continue
            alt = _relative_alt_m(event["value"])
            if alt is None:
                continue
            self.last_relative_alt_m = alt
            if alt <= target_m:
                logging.info("mission: landed at %.2f m", alt)
                return

    def _wait_for_first_heartbeat(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        for event in self.node:
            if time.monotonic() >= deadline:
                raise TimeoutError(f"no HEARTBEAT within {timeout:.0f}s -- is SITL running?")
            if event["type"] == "STOP":
                raise RuntimeError("STOP received before mission start")
            if event["type"] == "INPUT" and event["id"] == "heartbeat":
                logging.info("mission: vehicle heartbeat received")
                return

    # ---- mission steps -----------------------------------------------------

    def run(self) -> None:
        self._wait_for_first_heartbeat(timeout=COMMAND_TIMEOUT_SECS)

        self._send(
            "SET_MODE GUIDED",
            MAV_CMD_DO_SET_MODE,
            p1=MODE_FLAG_CUSTOM_ENABLED,
            p2=float(ARDUCOPTER_GUIDED_CUSTOM_MODE),
        )
        self._await_ack(COMMAND_TIMEOUT_SECS)

        self._send("ARM", MAV_CMD_COMPONENT_ARM_DISARM, p1=1.0)
        self._await_ack(COMMAND_TIMEOUT_SECS)

        self._send("TAKEOFF", MAV_CMD_NAV_TAKEOFF, p7=TAKEOFF_ALT_M)
        self._await_ack(COMMAND_TIMEOUT_SECS)
        self._await_alt_at_least(ALT_REACHED_M, timeout=ALT_TIMEOUT_SECS)

        logging.info("mission: hovering for %.1fs", HOVER_SECS)
        time.sleep(HOVER_SECS)

        self._send("LAND", MAV_CMD_NAV_LAND)
        self._await_ack(COMMAND_TIMEOUT_SECS)
        self._await_alt_below(LAND_DETECT_M, timeout=ALT_TIMEOUT_SECS)

        self._send("DISARM", MAV_CMD_COMPONENT_ARM_DISARM, p1=0.0)
        self._await_ack(COMMAND_TIMEOUT_SECS)

        # Marker for smoke / log-grep tests.
        print("mission: SUCCESS - takeoff + hover + land + disarm complete")


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    node = Node()
    try:
        Mission(node).run()
    except (TimeoutError, RuntimeError) as e:
        logging.error("mission: FAILED - %s", e)
        raise SystemExit(1) from e


if __name__ == "__main__":
    main()
