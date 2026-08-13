#!/usr/bin/env bash
#
# Boot ArduCopter SITL with reproducible flags so the mission demo
# always starts from the same home position. Forwards MAVLink on
# udp:127.0.0.1:14550 (the bridge's default).
#
# Prereqs (Ubuntu and macOS):
#
#   git clone --recursive https://github.com/ArduPilot/ardupilot.git ~/src/ardupilot
#   cd ~/src/ardupilot
#   Tools/environment_install/install-prereqs-<ubuntu|mac>.sh -y
#   ./waf configure --board sitl
#   ./waf copter
#   pip install --user MAVProxy   # optional; only needed if you want
#                                 # the MAVProxy console / map UI
#
# Then point ARDUPILOT_SRC at the clone (or pass it as the first
# positional argument):
#
#   ARDUPILOT_SRC=~/src/ardupilot ./scripts/start_sitl.sh
#
# Stop with Ctrl-C.

set -euo pipefail

ARDUPILOT_SRC="${1:-${ARDUPILOT_SRC:-}}"

if [ -z "$ARDUPILOT_SRC" ]; then
    cat >&2 <<'USAGE'
ERROR: ARDUPILOT_SRC not set.

Usage:
  ARDUPILOT_SRC=~/src/ardupilot ./scripts/start_sitl.sh
  # or:
  ./scripts/start_sitl.sh ~/src/ardupilot

Install ArduPilot from https://github.com/ArduPilot/ardupilot first.
USAGE
    exit 1
fi

if [ ! -x "$ARDUPILOT_SRC/Tools/autotest/sim_vehicle.py" ]; then
    echo "ERROR: $ARDUPILOT_SRC/Tools/autotest/sim_vehicle.py not found or not executable" >&2
    exit 1
fi

# Reproducible home: 37.7749 N, 122.4194 W (San Francisco), 0 m, 0 deg yaw.
HOME_LATLON="${MAVLINK_SITL_HOME:-37.7749,-122.4194,0,0}"

# --no-mavproxy: skip MAVProxy entirely; sim_vehicle exposes the
#                bare TCP/UDP MAVLink ports directly.
# --out=udp:127.0.0.1:14550: forward MAVLink to the bridge.
# -v ArduCopter: vehicle type.
# --frame=quad: Iris-style quadrotor.
exec "$ARDUPILOT_SRC/Tools/autotest/sim_vehicle.py" \
    -v ArduCopter \
    --frame=quad \
    --custom-location="$HOME_LATLON" \
    --out=udp:127.0.0.1:14550 \
    --no-mavproxy
