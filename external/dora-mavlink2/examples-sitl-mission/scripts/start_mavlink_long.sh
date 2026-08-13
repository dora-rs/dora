#!/usr/bin/env bash
# Boot dronekit-sitl + a simple Python forwarder for the long-mission demo.
#
# Topology:
#   dronekit-sitl ── tcp:5760 ──► dora bridge   (configured with ?proto=v1)
#                ── tcp:5762 ──► forward_to_qgc.py ──udp:14550──► QGC
#
# Why ?proto=v1: mavlink-rust 0.13's recv() is filtered by the connection's
# protocol_version. dronekit-sitl ships ArduCopter 3.3 which speaks V1 only;
# pinning the bridge to V2 (the bridge's default) made it silently drop
# every frame. The dataflow_long.yml endpoint URL passes ?proto=v1 so the
# bridge uses V1 framing on this link.
#
# Logs: /tmp/dronekit-sitl.log and /tmp/forward_to_qgc.log
# Stop everything: pkill -f 'dronekit-sitl|forward_to_qgc'

set -euo pipefail

HOME_LATLON="${MAVLINK_SITL_HOME:--35.363261,149.165230,584,353}"
SITL_LOG=/tmp/dronekit-sitl.log
FORWARD_LOG=/tmp/forward_to_qgc.log
SITL_BIN="${HOME}/.local/bin/dronekit-sitl"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FORWARDER="$SCRIPT_DIR/forward_to_qgc.py"

if ! command -v "$SITL_BIN" >/dev/null 2>&1; then
    echo "ERROR: $SITL_BIN not found. Run: pip install --user dronekit-sitl" >&2
    exit 1
fi
if [ ! -f "$FORWARDER" ]; then
    echo "ERROR: $FORWARDER not found." >&2
    exit 1
fi

# Kill any prior instances.
pkill -f 'dronekit-sitl|sitl-linux-copter' || true
pkill -f 'forward_to_qgc.py' || true
pkill -f 'mavproxy.py' || true
sleep 1

echo "[1/2] Booting dronekit-sitl ArduCopter at home=$HOME_LATLON"
"$SITL_BIN" copter --home="$HOME_LATLON" >"$SITL_LOG" 2>&1 &
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

sleep 2

echo
echo "Bring-up complete:"
echo "  dronekit-sitl     pid=$SITL_PID  (log: $SITL_LOG)"
echo "  forward_to_qgc.py pid=$FWD_PID  (log: $FORWARD_LOG)"
echo
echo "QGC          ← udp:14550 ← forward_to_qgc.py ← tcp:5762 ← dronekit-sitl"
echo "dora bridge  ← directly  ← tcp:5760 (?proto=v1) ← dronekit-sitl"
echo
echo "Next step: dora run dataflow_long.yml"
echo
echo "Stop everything:"
echo "  pkill -f 'dronekit-sitl|forward_to_qgc.py'"
