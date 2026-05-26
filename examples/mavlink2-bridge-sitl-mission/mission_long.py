"""Long-and-obvious closed-loop SITL mission for QGC visualization.

Extends the original `mission.py` (takeoff → hover → land) with:

  WAIT_HEARTBEAT → SET_GUIDED → ARM →
  TAKEOFF (30m) → REPOSITION square (4 waypoints, 100m sides) →
  CLIMB to 50m → HOVER 30s → LAND → DISARM → DONE

Total flight time ~3 minutes, traces a visible square on the QGC map,
then climbs and slowly descends. Designed to be obvious to a human
watching QGC; no human input required.

Falls back gracefully on old firmware: if DO_REPOSITION returns
TEMPORARILY_REJECTED / UNSUPPORTED / DENIED, the waypoint phase is
skipped and the mission continues with extended hover + tall LAND.

Tunables (env vars; defaults chosen for "obvious in QGC"):

  MAVLINK_SITL_TAKEOFF_ALT_M     30.0  initial cruise altitude (m)
  MAVLINK_SITL_HIGH_ALT_M        50.0  climb altitude after square (m)
  MAVLINK_SITL_HOVER_SECS        30.0  hover hold before LAND (s)
  MAVLINK_SITL_HOME_LAT          home lat (deg, signed)
  MAVLINK_SITL_HOME_LON          home lon (deg, signed)
  MAVLINK_SITL_LEG_M             100.0 length of each square side (m)
  MAVLINK_SITL_GROUND_SPEED      5.0   m/s for DO_REPOSITION
  MAVLINK_SITL_SKIP_REPOSITION   0     "1" disables the square (climb only)

Default home matches dronekit-sitl's built-in CMAC location
(-35.363261, 149.165230) so the waypoints are correct out of the box
without --home overrides.
"""

from __future__ import annotations

import logging
import math
import os
import time
from dataclasses import dataclass

import pyarrow as pa
from dora import Node

try:
    from pymavlink import mavutil  # type: ignore[import-not-found]
except Exception:  # pragma: no cover
    mavutil = None  # type: ignore[assignment]


_SIDE_PYMAVLINK = None


def _ensure_side_pymavlink():
    """Lazy-open a TCP-5763 pymavlink connection.

    Used for SET_MODE (because mavlink-rust 0.13's strict MavMode enum
    refuses base_mode=1) AND for REQUEST_DATA_STREAM (because the bridge
    doesn't auto-request streams, and ArduCopter 3.3 only emits HEARTBEAT
    by default — without this, GLOBAL_POSITION_INT never arrives and the
    mission times out waiting for altitude). Connection is reused."""
    global _SIDE_PYMAVLINK
    if _SIDE_PYMAVLINK is not None:
        return _SIDE_PYMAVLINK
    if mavutil is None:
        raise RuntimeError(
            "pymavlink is not installed; install with `pip install --user pymavlink`"
        )
    side_url = os.environ.get("MAVLINK_SITL_SIDECHAN", "tcp:127.0.0.1:5763")
    logging.info("mission: opening side-channel %s", side_url)
    side = mavutil.mavlink_connection(side_url, source_system=255, source_component=190)
    side.wait_heartbeat(timeout=10)
    # Request all data streams at 5 Hz so the bridge sees GLOBAL_POSITION_INT,
    # COMMAND_ACK, SYS_STATUS, etc. flow continuously.
    side.mav.request_data_stream_send(
        side.target_system, side.target_component,
        0,    # MAV_DATA_STREAM_ALL
        5,    # rate Hz
        1,    # start
    )
    logging.info("mission: requested data streams at 5Hz from side-channel")
    _SIDE_PYMAVLINK = side
    return side

# MAV_CMD enum values.
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_NAV_LAND = 21
MAV_CMD_DO_SET_MODE = 176
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_DO_REPOSITION = 192

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
MAV_RESULT_SOFT_FAIL = {1, 2, 3, 4}  # treat as "skip leg, keep flying"

ARDUCOPTER_GUIDED_CUSTOM_MODE = 4
MODE_FLAG_CUSTOM_ENABLED = 1.0
# When True, send the legacy MAVLink SET_MODE message (msg id 11) instead
# of MAV_CMD_DO_SET_MODE (cmd 176). Required for ArduCopter <= 3.5
# (dronekit-sitl ships 3.3) which returns ACK=UNSUPPORTED for cmd 176.
USE_LEGACY_SET_MODE = os.environ.get("MAVLINK_SITL_LEGACY_SET_MODE", "1") == "1"

# Tunables.
TARGET_SYSTEM = int(os.environ.get("MAVLINK_SITL_TARGET_SYSTEM", "1"))
TARGET_COMPONENT = int(os.environ.get("MAVLINK_SITL_TARGET_COMPONENT", "1"))
TAKEOFF_ALT_M = float(os.environ.get("MAVLINK_SITL_TAKEOFF_ALT_M", "30.0"))
HIGH_ALT_M = float(os.environ.get("MAVLINK_SITL_HIGH_ALT_M", "50.0"))
HOVER_SECS = float(os.environ.get("MAVLINK_SITL_HOVER_SECS", "30.0"))
COMMAND_TIMEOUT_SECS = float(os.environ.get("MAVLINK_SITL_COMMAND_TIMEOUT", "15.0"))
ALT_TIMEOUT_SECS = float(os.environ.get("MAVLINK_SITL_ALT_TIMEOUT", "60.0"))
WARMUP_SECS = float(os.environ.get("MAVLINK_SITL_WARMUP_SECS", "0.0"))
FORCE_ARM = os.environ.get("MAVLINK_SITL_FORCE_ARM", "0") == "1"
SKIP_REPOSITION = os.environ.get("MAVLINK_SITL_SKIP_REPOSITION", "0") == "1"
WAYPOINT_TIMEOUT_SECS = float(os.environ.get("MAVLINK_SITL_WAYPOINT_TIMEOUT", "45.0"))

HOME_LAT = float(os.environ.get("MAVLINK_SITL_HOME_LAT", "-35.363261"))
HOME_LON = float(os.environ.get("MAVLINK_SITL_HOME_LON", "149.165230"))
LEG_M = float(os.environ.get("MAVLINK_SITL_LEG_M", "100.0"))
GROUND_SPEED = float(os.environ.get("MAVLINK_SITL_GROUND_SPEED", "5.0"))

ALT_REACHED_FRACTION = 0.85
LAND_DETECT_M = 0.5

# 1 deg latitude ~= 111 320 m at the equator; small variation with latitude
# is negligible at the precision we need.
METERS_PER_DEG_LAT = 111_320.0


def _meters_per_deg_lon(lat_deg: float) -> float:
    return METERS_PER_DEG_LAT * math.cos(math.radians(lat_deg))


def _offset_latlon(lat: float, lon: float, north_m: float, east_m: float) -> tuple[float, float]:
    new_lat = lat + north_m / METERS_PER_DEG_LAT
    new_lon = lon + east_m / _meters_per_deg_lon(lat)
    return new_lat, new_lon


@dataclass
class Pending:
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
            pa.array([0], type=pa.uint8()),
        ],
        names=[
            "param1", "param2", "param3", "param4",
            "param5", "param6", "param7",
            "command", "target_system", "target_component", "confirmation",
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


def _global_latlon(global_position_int: pa.StructArray) -> tuple[float, float] | None:
    row = _first_row(global_position_int)
    lat = row.get("lat")
    lon = row.get("lon")
    if lat is None or lon is None:
        return None
    return lat / 1e7, lon / 1e7


class Mission:
    def __init__(self, node: Node) -> None:
        self.node = node
        self.pending: Pending | None = None
        self.last_relative_alt_m: float | None = None
        self.last_latlon: tuple[float, float] | None = None

    # ---- low-level helpers -------------------------------------------------

    def _send(self, description: str, command: int, **params: float) -> None:
        cmd = _make_command_long(command, **params)
        self.node.send_output("command_long_cmd", cmd)
        self.pending = Pending(command=command, description=description, sent_at=time.monotonic())
        logging.info("mission: -> %s (cmd=%d)", description, command)

    def _send_set_mode(self, custom_mode: int, base_mode: int = 1) -> None:
        """Send the legacy MAVLink SET_MODE message (msg id 11).

        The dora bridge can carry the MAVLink SET_MODE message in
        principle, but mavlink-rust 0.13's generated `SET_MODE_DATA.base_mode`
        is the strict `MavMode` enum (no variant for the bare bit 0x01 used
        by ArduCopter), so any `base_mode=1` payload fails decoding inside
        the bridge and is silently dropped. Side-channel via pymavlink on
        TCP 5763 (a parallel SITL UART) bypasses that limitation cleanly —
        the autopilot's MAVLink stack applies the mode regardless of which
        port the message came in on, and HEARTBEATs continue flowing over
        the bridge so we still observe the mode change end-to-end."""
        side = _ensure_side_pymavlink()
        side.mav.set_mode_send(TARGET_SYSTEM, base_mode, custom_mode)
        logging.info(
            "mission: -> SET_MODE (side-channel) custom_mode=%d base_mode=%d",
            custom_mode, base_mode,
        )
        # Keep the dora bridge input plumbed too so future autopilots that
        # support MAV_CMD_DO_SET_MODE just work without this hack.
        sm = pa.StructArray.from_arrays(
            [
                pa.array([custom_mode], type=pa.uint32()),
                pa.array([TARGET_SYSTEM], type=pa.uint8()),
                pa.array([base_mode], type=pa.uint8()),
            ],
            names=["custom_mode", "target_system", "base_mode"],
        )
        try:
            self.node.send_output("set_mode_cmd", sm)
        except Exception:
            pass

    def _await_ack(self, timeout: float, *, soft_fail_ok: bool = False) -> int:
        """Block until COMMAND_ACK for self.pending arrives.

        Returns the MAV_RESULT code. Raises on hard fail (non-soft, non-accepted)
        unless soft_fail_ok=True, in which case soft-fail codes are returned to
        the caller without raising."""
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
                ll = _global_latlon(event["value"])
                if ll is not None:
                    self.last_latlon = ll
                continue
            if input_id == "command_ack":
                row = _first_row(event["value"])
                if row.get("command") != self.pending.command:
                    continue
                result = row.get("result", -1)
                name = MAV_RESULT_NAMES.get(result, f"UNKNOWN({result})")
                logging.info(
                    "mission: <- ACK %s [%s, %.1fs]",
                    self.pending.description, name,
                    time.monotonic() - self.pending.sent_at,
                )
                self.pending = None
                if result == MAV_RESULT_ACCEPTED:
                    return result
                if soft_fail_ok and result in MAV_RESULT_SOFT_FAIL:
                    return result
                raise RuntimeError(
                    f"{self.pending.description if self.pending else name}: "
                    f"ACK = {name} (result={result})"
                )
        raise RuntimeError("event stream ended before ACK")

    def _await_alt_at_least(self, target_m: float, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        for event in self.node:
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"altitude did not reach {target_m:.1f} m within {timeout:.0f}s "
                    f"(last={self.last_relative_alt_m})"
                )
            if event["type"] == "STOP":
                raise RuntimeError("STOP received mid-mission")
            if event["type"] != "INPUT" or event["id"] != "global_position_int":
                continue
            alt = _relative_alt_m(event["value"])
            ll = _global_latlon(event["value"])
            if ll is not None:
                self.last_latlon = ll
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
                    f"(last={self.last_relative_alt_m})"
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

    def _await_reach_latlon(self, target_lat: float, target_lon: float,
                            tolerance_m: float, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        for event in self.node:
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"vehicle did not reach ({target_lat:.5f}, {target_lon:.5f}) "
                    f"within {timeout:.0f}s (last_latlon={self.last_latlon})"
                )
            if event["type"] == "STOP":
                raise RuntimeError("STOP received mid-mission")
            if event["type"] != "INPUT" or event["id"] != "global_position_int":
                continue
            alt = _relative_alt_m(event["value"])
            ll = _global_latlon(event["value"])
            if alt is not None:
                self.last_relative_alt_m = alt
            if ll is None:
                continue
            self.last_latlon = ll
            cur_lat, cur_lon = ll
            dn = (cur_lat - target_lat) * METERS_PER_DEG_LAT
            de = (cur_lon - target_lon) * _meters_per_deg_lon(cur_lat)
            dist_m = math.hypot(dn, de)
            if dist_m <= tolerance_m:
                logging.info(
                    "mission: reached waypoint dist=%.1f m alt=%.1f m",
                    dist_m, alt or -1,
                )
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

    def _wait_for_custom_mode(self, target_custom_mode: int, timeout: float) -> None:
        """Drain events until a HEARTBEAT reports custom_mode == target."""
        deadline = time.monotonic() + timeout
        for event in self.node:
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"vehicle did not reach custom_mode={target_custom_mode} in {timeout:.0f}s"
                )
            if event["type"] == "STOP":
                raise RuntimeError("STOP received during mode-wait")
            if event["type"] != "INPUT":
                continue
            if event["id"] == "global_position_int":
                self.last_relative_alt_m = _relative_alt_m(event["value"])
                ll = _global_latlon(event["value"])
                if ll is not None:
                    self.last_latlon = ll
                continue
            if event["id"] == "heartbeat":
                row = _first_row(event["value"])
                cm = row.get("custom_mode")
                if cm == target_custom_mode:
                    logging.info("mission: vehicle is in custom_mode=%d", cm)
                    return

    # ---- mission steps -----------------------------------------------------

    def _try_reposition(self, label: str, lat: float, lon: float, alt_m: float) -> bool:
        """Send DO_REPOSITION and wait for arrival. Returns False on soft-fail
        (caller skips remaining waypoints). Raises on timeout."""
        self._send(
            f"REPOSITION {label} ({lat:.5f}, {lon:.5f}, {alt_m:.0f}m)",
            MAV_CMD_DO_REPOSITION,
            p1=GROUND_SPEED,
            p2=0.0,
            p3=0.0,
            p4=float("nan"),
            p5=lat,
            p6=lon,
            p7=alt_m,
        )
        result = self._await_ack(COMMAND_TIMEOUT_SECS, soft_fail_ok=True)
        if result != MAV_RESULT_ACCEPTED:
            logging.warning(
                "mission: REPOSITION %s soft-failed (%s); skipping remaining waypoints",
                label, MAV_RESULT_NAMES.get(result, result),
            )
            return False
        # Wait until we get within 5m horizontal of the target.
        self._await_reach_latlon(lat, lon, tolerance_m=5.0, timeout=WAYPOINT_TIMEOUT_SECS)
        return True

    def run(self) -> None:
        self._wait_for_first_heartbeat(timeout=COMMAND_TIMEOUT_SECS)

        if WARMUP_SECS > 0:
            logging.info("mission: warmup for %.1fs (EKF/GPS settle)", WARMUP_SECS)
            t0 = time.monotonic()
            for event in self.node:
                if time.monotonic() - t0 >= WARMUP_SECS:
                    break
                if event["type"] == "INPUT" and event["id"] == "global_position_int":
                    alt = _relative_alt_m(event["value"])
                    if alt is not None:
                        self.last_relative_alt_m = alt
                    ll = _global_latlon(event["value"])
                    if ll is not None:
                        self.last_latlon = ll

        if USE_LEGACY_SET_MODE:
            # MavModeFlag::CUSTOM_MODE_ENABLED bit (0x01)
            self._send_set_mode(custom_mode=ARDUCOPTER_GUIDED_CUSTOM_MODE, base_mode=1)
            # Wait until a HEARTBEAT confirms GUIDED mode is active. Without
            # this, ArduCopter 3.3 sometimes ARMs in STABILIZE and rejects
            # the subsequent TAKEOFF with ACK=FAILED.
            self._wait_for_custom_mode(ARDUCOPTER_GUIDED_CUSTOM_MODE, timeout=10.0)
        else:
            self._send(
                "SET_MODE GUIDED",
                MAV_CMD_DO_SET_MODE,
                p1=MODE_FLAG_CUSTOM_ENABLED,
                p2=float(ARDUCOPTER_GUIDED_CUSTOM_MODE),
            )
            self._await_ack(COMMAND_TIMEOUT_SECS)

        arm_kwargs: dict[str, float] = {"p1": 1.0}
        if FORCE_ARM:
            arm_kwargs["p2"] = 21196.0
        self._send("ARM", MAV_CMD_COMPONENT_ARM_DISARM, **arm_kwargs)
        self._await_ack(COMMAND_TIMEOUT_SECS)

        self._send("TAKEOFF", MAV_CMD_NAV_TAKEOFF, p7=TAKEOFF_ALT_M)
        self._await_ack(COMMAND_TIMEOUT_SECS)
        self._await_alt_at_least(ALT_REACHED_FRACTION * TAKEOFF_ALT_M, ALT_TIMEOUT_SECS)

        # Square pattern, 4 waypoints. Default 100m sides at takeoff altitude.
        if not SKIP_REPOSITION:
            wp_n  = _offset_latlon(HOME_LAT, HOME_LON, north_m=LEG_M, east_m=0.0)
            wp_ne = _offset_latlon(HOME_LAT, HOME_LON, north_m=LEG_M, east_m=LEG_M)
            wp_e  = _offset_latlon(HOME_LAT, HOME_LON, north_m=0.0,    east_m=LEG_M)
            wp_home_high = (HOME_LAT, HOME_LON)
            for label, (lat, lon), alt_m in [
                ("NORTH",      wp_n,             TAKEOFF_ALT_M),
                ("NORTH-EAST", wp_ne,            TAKEOFF_ALT_M),
                ("EAST",       wp_e,             TAKEOFF_ALT_M),
                ("HOME-HIGH",  wp_home_high,     HIGH_ALT_M),
            ]:
                ok = self._try_reposition(label, lat, lon, alt_m)
                if not ok:
                    break
        else:
            logging.info("mission: SKIP_REPOSITION=1 -> climb only")
            # Reuse TAKEOFF for an additional climb.
            self._send("CLIMB", MAV_CMD_NAV_TAKEOFF, p7=HIGH_ALT_M)
            try:
                self._await_ack(COMMAND_TIMEOUT_SECS, soft_fail_ok=True)
                self._await_alt_at_least(ALT_REACHED_FRACTION * HIGH_ALT_M, ALT_TIMEOUT_SECS)
            except (RuntimeError, TimeoutError) as e:
                logging.warning("mission: CLIMB skipped (%s)", e)

        logging.info(
            "mission: hovering for %.1fs at alt~%sm",
            HOVER_SECS, self.last_relative_alt_m,
        )
        time.sleep(HOVER_SECS)

        self._send("LAND", MAV_CMD_NAV_LAND)
        self._await_ack(COMMAND_TIMEOUT_SECS)
        self._await_alt_below(LAND_DETECT_M, timeout=max(ALT_TIMEOUT_SECS, HIGH_ALT_M * 2))

        time.sleep(3.0)
        disarm_kwargs: dict[str, float] = {"p1": 0.0}
        if FORCE_ARM:
            disarm_kwargs["p2"] = 21196.0
        self._send("DISARM", MAV_CMD_COMPONENT_ARM_DISARM, **disarm_kwargs)
        try:
            self._await_ack(COMMAND_TIMEOUT_SECS)
        except RuntimeError as e:
            logging.info("mission: DISARM rejected (likely auto-disarmed): %s", e)
            self.pending = None

        print("mission: SUCCESS - long mission complete (square + climb + hover + land)")


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
