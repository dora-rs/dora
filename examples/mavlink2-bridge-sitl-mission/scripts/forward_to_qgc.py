#!/usr/bin/env python3
"""Forward MAVLink from dronekit-sitl's secondary TCP port to QGC's UDP port.

dronekit-sitl exposes multiple TCP ports (5760, 5762, 5763 for UART 0/2/3).
The dora bridge connects to 5760; QGC listens on UDP 14550. This script
bridges TCP 5762 (a parallel autopilot port) ↔ UDP 14550 so QGC sees the
same vehicle without competing with the dora bridge for TCP 5760.

No re-encoding — `recv_msg()` returns the original frame and `get_msgbuf()`
gives the raw bytes, so V1/V2 are preserved as-sent by ArduPilot.

Usage:
    python3 forward_to_qgc.py            # default 5762 -> 127.0.0.1:14550
    SRC_TCP_PORT=5762 DST_UDP=127.0.0.1:14550 python3 forward_to_qgc.py
"""

import os
import sys
import time
from pymavlink import mavutil

SRC_TCP_HOST = os.environ.get("SRC_TCP_HOST", "127.0.0.1")
SRC_TCP_PORT = int(os.environ.get("SRC_TCP_PORT", "5762"))
DST_UDP = os.environ.get("DST_UDP", "127.0.0.1:14550")


def main() -> int:
    print(
        f"[forward_to_qgc] tcp:{SRC_TCP_HOST}:{SRC_TCP_PORT}  ->  udpout:{DST_UDP}",
        flush=True,
    )
    src = mavutil.mavlink_connection(
        f"tcp:{SRC_TCP_HOST}:{SRC_TCP_PORT}",
        autoreconnect=True,
    )
    dst = mavutil.mavlink_connection(f"udpout:{DST_UDP}")
    print("[forward_to_qgc] connected to TCP source; awaiting frames", flush=True)
    n = 0
    while True:
        msg = src.recv_msg()
        if msg is None:
            time.sleep(0.005)
            continue
        try:
            dst.write(msg.get_msgbuf())
            n += 1
            if n in (1, 10, 100) or n % 1000 == 0:
                print(f"[forward_to_qgc] forwarded {n} frames", flush=True)
        except Exception as e:
            print(f"[forward_to_qgc] write failed: {e}", flush=True, file=sys.stderr)
            time.sleep(0.05)


if __name__ == "__main__":
    sys.exit(main() or 0)
