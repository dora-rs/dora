#!/usr/bin/env bash
# Boot dronekit-sitl rover-2.50 + Python TCP→UDP forwarder for QGC.
#
# Topology:
#   dronekit-sitl rover ── tcp:5760 ──► dora bridge   (?proto=v1)
#                       ── tcp:5762 ──► forward_to_qgc.py ──udp:14550──► QGC
#
# Same pattern as start_mavlink_long.sh but loads the rover-2.50 binary
# instead of copter-3.3.

set -euo pipefail

HOME_LATLON="${MAVLINK_SITL_HOME:--35.363261,149.165230,584,353}"
SITL_LOG=/tmp/dronekit-rover.log
FORWARD_LOG=/tmp/forward_to_qgc.log
SITL_BIN="${HOME}/.local/bin/dronekit-sitl"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FORWARDER="$SCRIPT_DIR/forward_to_qgc.py"

if ! command -v "$SITL_BIN" >/dev/null 2>&1; then
    echo "ERROR: $SITL_BIN not found. Run: pip install --user dronekit-sitl" >&2
    exit 1
fi

# Hard-stop any prior SITL.
pkill -f 'dronekit-sitl|sitl-linux-' || true
pkill -f 'forward_to_qgc.py' || true
pkill -f 'mavproxy.py' || true
# Force-kill any apm process directly (dronekit-sitl wrapper may be gone
# but the apm child can hang on to TCP ports).
pkill -KILL -f '/.dronekit/sitl/.*/apm' || true
sleep 2

echo "[1/2] Booting dronekit-sitl rover-2.50 at home=$HOME_LATLON"
"$SITL_BIN" rover-2.50 --home="$HOME_LATLON" >"$SITL_LOG" 2>&1 &
SITL_PID=$!

for i in $(seq 1 60); do
    if (echo > /dev/tcp/127.0.0.1/5760) 2>/dev/null; then
        echo "       SITL TCP 5760 ready (after ${i}s)"
        break
    fi
    sleep 1
done
for i in $(seq 1 20); do
    (echo > /dev/tcp/127.0.0.1/5762) 2>/dev/null && break
    sleep 0.5
done

echo "[2/2] Starting QGC forwarder: tcp:5762 -> udp:14550"
python3 "$FORWARDER" >"$FORWARD_LOG" 2>&1 &
FWD_PID=$!

sleep 3

echo
echo "Bring-up complete:"
echo "  dronekit-sitl     pid=$SITL_PID  (log: $SITL_LOG)"
echo "  forward_to_qgc.py pid=$FWD_PID  (log: $FORWARD_LOG)"
echo
echo "QGC          ← udp:14550 ← forward_to_qgc.py ← tcp:5762 ← dronekit-sitl rover"
echo "dora bridge  ← directly  ← tcp:5760 (?proto=v1) ← dronekit-sitl rover"
echo
echo "Next step: dora run dataflow_rover.yml"
echo
echo "Stop everything:"
echo "  pkill -f 'dronekit-sitl|forward_to_qgc.py'"
