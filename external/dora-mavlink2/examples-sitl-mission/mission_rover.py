"""Long-and-obvious rover mission via the dora mavlink2-bridge.

Why RC override and not GUIDED waypoints:
  dronekit-sitl ships ArduRover 2.50, which rejects COMMAND_LONG ARM
  (cmd 400), DO_REPOSITION (cmd 192), and NAV_WAYPOINT (cmd 16) — every
  ACK comes back result=3 (UNSUPPORTED). The only movement path that
  works on this firmware is RC_CHANNELS_OVERRIDE, which simulates an
  RC transmitter feeding steering + throttle. ArduRover auto-arms on
  throttle in MANUAL mode, so we can drive a square pattern without
  needing an explicit ARM command.

Drives a clockwise square (forward → right turn → forward → ...) by
alternating between forward-throttle / straight-steering and zero-throttle
/ hard-right-steering phases. Phase durations are tuned to the simulated
rover's nominal speed and turn rate; both are env-tunable. Telemetry
flows through the dora bridge as usual, so QGC sees the trajectory live.

State machine:
   WAIT_HEARTBEAT
   SET_MODE MANUAL (side-channel; mavlink-rust 0.13's strict MavMode
                    enum can't represent base_mode=1)
   for leg in [N, E, S, W]:
       drive forward LEG_SECS
       turn right TURN_SECS
   stop (RC override neutral)
   DONE

Tunables:
   MAVLINK_SITL_LEG_SECS        time per straight leg (default 25.0s)
   MAVLINK_SITL_TURN_SECS       time per 90-deg turn (default 4.0s)
   MAVLINK_SITL_TICK_HZ         RC override resend rate (default 10 Hz)
   MAVLINK_SITL_THROTTLE_PWM    forward throttle PWM (default 1900)
   MAVLINK_SITL_NEUTRAL_PWM     neutral PWM (default 1500)
   MAVLINK_SITL_STEER_RIGHT_PWM hard-right steering PWM (default 1900)
"""

from __future__ import annotations

import logging
import math
import os
import time

import pyarrow as pa
from dora import Node

try:
    from pymavlink import mavutil  # type: ignore[import-not-found]
except Exception:  # pragma: no cover
    mavutil = None  # type: ignore[assignment]


# Rover modes (ArduRover firmware): 0=MANUAL, 4=HOLD, 10=AUTO, 11=RTL,
# 15=GUIDED. We use MANUAL because it's the only mode that takes
# RC_CHANNELS_OVERRIDE in rover-2.50.
ARDUROVER_MANUAL_MODE = 0

TARGET_SYSTEM = int(os.environ.get("MAVLINK_SITL_TARGET_SYSTEM", "1"))
TARGET_COMPONENT = int(os.environ.get("MAVLINK_SITL_TARGET_COMPONENT", "1"))

LEG_SECS = float(os.environ.get("MAVLINK_SITL_LEG_SECS", "25.0"))
TURN_SECS = float(os.environ.get("MAVLINK_SITL_TURN_SECS", "4.0"))
TICK_HZ = float(os.environ.get("MAVLINK_SITL_TICK_HZ", "10.0"))
THROTTLE_PWM = int(os.environ.get("MAVLINK_SITL_THROTTLE_PWM", "1900"))
NEUTRAL_PWM = int(os.environ.get("MAVLINK_SITL_NEUTRAL_PWM", "1500"))
STEER_RIGHT_PWM = int(os.environ.get("MAVLINK_SITL_STEER_RIGHT_PWM", "1900"))


_SIDE_PYMAVLINK = None


def _ensure_side_pymavlink():
    """Side-channel pymavlink connection on TCP 5763 for SET_MODE +
    REQUEST_DATA_STREAM. See `_send_set_mode` in mission_long.py for the
    rationale (mavlink-rust 0.13 strict MavMode + missing data streams)."""
    global _SIDE_PYMAVLINK
    if _SIDE_PYMAVLINK is not None:
        return _SIDE_PYMAVLINK
    if mavutil is None:
        raise RuntimeError("pymavlink missing; pip install --user pymavlink")
    side_url = os.environ.get("MAVLINK_SITL_SIDECHAN", "tcp:127.0.0.1:5763")
    logging.info("mission: opening side-channel %s", side_url)
    side = mavutil.mavlink_connection(side_url, source_system=255, source_component=190)
    side.wait_heartbeat(timeout=15)
    side.mav.request_data_stream_send(side.target_system, side.target_component, 0, 5, 1)
    _SIDE_PYMAVLINK = side
    return side


def _make_rc_override(steering_pwm: int, throttle_pwm: int) -> pa.StructArray:
    """RC_CHANNELS_OVERRIDE. ArduRover default: ch1=steering, ch3=throttle.
    All other channels neutral. UINT16_MAX would mean "release"; we send
    explicit values so the autopilot uses our overrides on every channel."""
    return pa.StructArray.from_arrays(
        [
            pa.array([TARGET_SYSTEM], type=pa.uint8()),
            pa.array([TARGET_COMPONENT], type=pa.uint8()),
            pa.array([steering_pwm], type=pa.uint16()),
            pa.array([NEUTRAL_PWM], type=pa.uint16()),
            pa.array([throttle_pwm], type=pa.uint16()),
            pa.array([NEUTRAL_PWM], type=pa.uint16()),
            pa.array([NEUTRAL_PWM], type=pa.uint16()),
            pa.array([NEUTRAL_PWM], type=pa.uint16()),
            pa.array([NEUTRAL_PWM], type=pa.uint16()),
            pa.array([NEUTRAL_PWM], type=pa.uint16()),
        ],
        names=[
            "target_system", "target_component",
            "chan1_raw", "chan2_raw", "chan3_raw", "chan4_raw",
            "chan5_raw", "chan6_raw", "chan7_raw", "chan8_raw",
        ],
    )


def _first_row(struct_array: pa.StructArray) -> dict:
    rows = struct_array.to_pylist()
    return rows[0] if rows else {}


def _global_latlon(global_position_int) -> tuple[float, float] | None:
    row = _first_row(global_position_int)
    lat = row.get("lat")
    lon = row.get("lon")
    if lat is None or lon is None:
        return None
    return lat / 1e7, lon / 1e7


class RoverMission:
    def __init__(self, node: Node) -> None:
        self.node = node
        self.last_latlon: tuple[float, float] | None = None
        self.start_latlon: tuple[float, float] | None = None

    def _wait_for_first_heartbeat(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        for event in self.node:
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"no HEARTBEAT within {timeout:.0f}s — is SITL running?"
                )
            if event["type"] == "STOP":
                raise RuntimeError("STOP received before mission start")
            if event["type"] == "INPUT" and event["id"] == "heartbeat":
                logging.info("mission: vehicle heartbeat received")
                return

    def _drain_for(self, secs: float, *, send_rc=None,
                   side_pwm: tuple[int, int] | None = None) -> None:
        """Consume events for `secs` while pumping RC overrides at TICK_HZ.

        Sends through both the dora bridge (rc_channels_override_cmd) AND
        the side-channel pymavlink connection. mavlink-rust 0.13 may
        encode RC_CHANNELS_OVERRIDE in a way ArduRover 2.50 doesn't
        accept; the side-channel is the proven path."""
        deadline = time.monotonic() + secs
        next_send = time.monotonic()
        period = 1.0 / TICK_HZ
        side = _ensure_side_pymavlink() if side_pwm is not None else None
        for event in self.node:
            now = time.monotonic()
            if now >= next_send:
                if send_rc is not None:
                    try:
                        self.node.send_output("rc_channels_override_cmd", send_rc)
                    except Exception:
                        pass
                if side is not None and side_pwm is not None:
                    steer, throttle = side_pwm
                    side.mav.rc_channels_override_send(
                        side.target_system, side.target_component,
                        steer, NEUTRAL_PWM, throttle, NEUTRAL_PWM,
                        NEUTRAL_PWM, NEUTRAL_PWM, NEUTRAL_PWM, NEUTRAL_PWM,
                    )
                next_send = now + period
            if now >= deadline:
                return
            if event["type"] == "STOP":
                raise RuntimeError("STOP received during leg")
            if event["type"] == "INPUT" and event["id"] == "global_position_int":
                ll = _global_latlon(event["value"])
                if ll is not None:
                    self.last_latlon = ll
                    if self.start_latlon is None:
                        self.start_latlon = ll

    def _drive_forward(self, secs: float, label: str) -> None:
        rc = _make_rc_override(NEUTRAL_PWM, THROTTLE_PWM)
        before = self.last_latlon
        logging.info("mission: drive %s for %.1fs (steering=neutral throttle=%d)",
                     label, secs, THROTTLE_PWM)
        self._drain_for(secs, send_rc=rc, side_pwm=(NEUTRAL_PWM, THROTTLE_PWM))
        if before and self.last_latlon:
            dn = (self.last_latlon[0] - before[0]) * 111_320.0
            de = (self.last_latlon[1] - before[1]) * 111_320.0 * \
                 math.cos(math.radians(self.last_latlon[0]))
            logging.info("mission: leg %s done dN=%.1fm dE=%.1fm",
                         label, dn, de)

    def _turn_right(self, secs: float, label: str) -> None:
        rc = _make_rc_override(STEER_RIGHT_PWM, THROTTLE_PWM)
        logging.info("mission: turn right for %.1fs (%s)", secs, label)
        self._drain_for(secs, send_rc=rc, side_pwm=(STEER_RIGHT_PWM, THROTTLE_PWM))

    def _stop(self) -> None:
        rc = _make_rc_override(NEUTRAL_PWM, NEUTRAL_PWM)
        logging.info("mission: stop (RC neutral)")
        for _ in range(5):
            self.node.send_output("rc_channels_override_cmd", rc)
            time.sleep(0.05)

    def run(self) -> None:
        self._wait_for_first_heartbeat(timeout=15.0)
        side = _ensure_side_pymavlink()
        logging.info("mission: -> SET_MODE MANUAL (custom=%d)", ARDUROVER_MANUAL_MODE)
        side.mav.set_mode_send(side.target_system, 1, ARDUROVER_MANUAL_MODE)
        time.sleep(1.0)

        # Drive square: 4 forward legs + 4 right turns
        legs = [("NORTH", "E"), ("EAST", "S"), ("SOUTH", "W"), ("WEST", "N")]
        for fwd_label, turn_label in legs:
            self._drive_forward(LEG_SECS, fwd_label)
            self._turn_right(TURN_SECS, f"to {turn_label}")
        self._stop()

        if self.start_latlon and self.last_latlon:
            dn = (self.last_latlon[0] - self.start_latlon[0]) * 111_320.0
            de = (self.last_latlon[1] - self.start_latlon[1]) * 111_320.0 * \
                 math.cos(math.radians(self.last_latlon[0]))
            logging.info(
                "mission: total displacement from start: dN=%.1fm dE=%.1fm",
                dn, de,
            )
        print("mission: SUCCESS - rover square drive complete")


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    node = Node()
    try:
        RoverMission(node).run()
    except (TimeoutError, RuntimeError) as e:
        logging.error("mission: FAILED - %s", e)
        raise SystemExit(1) from e


if __name__ == "__main__":
    main()
