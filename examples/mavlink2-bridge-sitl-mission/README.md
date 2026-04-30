# MAVLink 2 bridge — SITL closed-loop mission

A dora dataflow that drives [ArduPilot Copter SITL](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)
through a full takeoff → hover → land → disarm cycle, entirely from a
**Python dora node** talking MAVLink through the bridge. Demonstrates
the dora ↔ MAVLink loop with real autopilot physics.

```
        ┌────────────────────────────────────────┐
        │  ArduCopter SITL (separate process)    │
        │  udp:14550                             │
        └─────────────────▲──────────────────────┘
                          │ MAVLink 2
                ┌─────────┴──────────┐
                │ dora-mavlink2-     │
                │ bridge-node        │
                └─┬────────▲─────────┘
       Arrow:     │        │   Arrow:
   command_long_  │        │   heartbeat /
   cmd            │        │   command_ack /
                  ▼        │   global_position_int
                ┌──────────┴─────┐
                │  mission.py    │   (Python state machine)
                └────────────────┘
```

> [!IMPORTANT]
> This example is **local-only**. SITL is heavy, requires a one-time
> install, and is intentionally not part of the smoke suite or CI.
> Develops verify the bridge against real autopilot semantics by
> running the demo manually before pushing changes that touch
> command-side code.

## Platform support

| Platform | Status | Notes |
|----------|--------|-------|
| Ubuntu 22.04 / 24.04 | First-class | Upstream ArduPilot dev target |
| macOS (ARM64 / Intel) | Supported | Needs `brew install cmake gawk`; SITL ~10–15% slower than Linux |
| Windows | **Not supported** | ArduPilot SITL on Windows requires WSL; the bridge + dataflow themselves work, but `scripts/start_sitl.sh` does not |

## One-time setup

### Ubuntu

```bash
git clone --recursive https://github.com/ArduPilot/ardupilot.git ~/src/ardupilot
cd ~/src/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
./waf configure --board sitl
./waf copter
```

### macOS

```bash
brew install cmake gawk pkg-config
git clone --recursive https://github.com/ArduPilot/ardupilot.git ~/src/ardupilot
cd ~/src/ardupilot
Tools/environment_install/install-prereqs-mac.sh -y
./waf configure --board sitl
./waf copter
```

The build takes 5–10 minutes the first time; subsequent rebuilds are
incremental.

## Run the demo

Two terminals:

**Terminal 1 — boot SITL:**

```bash
ARDUPILOT_SRC=~/src/ardupilot \
  ./examples/mavlink2-bridge-sitl-mission/scripts/start_sitl.sh
```

Wait until SITL prints `APM: EKF3 IMU0 is using GPS` (~10 s). It is
now broadcasting MAVLink on `udp:127.0.0.1:14550`.

**Terminal 2 — run the dora dataflow:**

```bash
dora run examples/mavlink2-bridge-sitl-mission/dataflow.yml --uv --stop-after 90s
```

`--uv` provisions a venv with `pyarrow` + `dora-rs` for the Python
mission node.

## What success looks like

```
mission: vehicle heartbeat received
mission: -> SET_MODE GUIDED (cmd=176)
mission: <- ACK SET_MODE GUIDED [ACCEPTED, 0.1s]
mission: -> ARM (cmd=400)
mission: <- ACK ARM [ACCEPTED, 0.1s]
mission: -> TAKEOFF (cmd=22)
mission: <- ACK TAKEOFF [ACCEPTED, 0.1s]
mission: reached 8.51 m (target 8.50)
mission: hovering for 5.0s
mission: -> LAND (cmd=21)
mission: <- ACK LAND [ACCEPTED, 0.0s]
mission: landed at 0.04 m
mission: -> DISARM (cmd=400)
mission: <- ACK DISARM [ACCEPTED, 0.1s]
mission: SUCCESS - takeoff + hover + land + disarm complete
```

The final `mission: SUCCESS` line is the canonical pass marker — pipe
to `grep` if you want to script success/failure detection.

## Tunables (env vars)

| Variable | Default | What it controls |
|----------|---------|------------------|
| `MAVLINK_SITL_TARGET_SYSTEM` | `1` | MAVLink system_id of the autopilot |
| `MAVLINK_SITL_TARGET_COMPONENT` | `1` | MAVLink component_id of the autopilot |
| `MAVLINK_SITL_TAKEOFF_ALT_M` | `10.0` | Target altitude after takeoff (m) |
| `MAVLINK_SITL_HOVER_SECS` | `5.0` | How long to hover before landing |
| `MAVLINK_SITL_COMMAND_TIMEOUT` | `15.0` | Per-command ACK timeout (s) |
| `MAVLINK_SITL_ALT_TIMEOUT` | `30.0` | Takeoff/land altitude-progress timeout (s) |
| `MAVLINK_SITL_HOME` | `37.7749,-122.4194,0,0` | Home `lat,lon,alt,yaw` for SITL |

## Failure modes

| Symptom | Likely cause |
|---------|--------------|
| `no HEARTBEAT within 15s` | SITL not running, or it's forwarding on a port other than 14550 |
| `ARM: ACK = TEMPORARILY_REJECTED` | EKF not yet healthy. Wait longer after boot before launching the dataflow, or raise `MAVLINK_SITL_COMMAND_TIMEOUT`. |
| `altitude did not reach … within 30s` | SITL physics paused (Ctrl-Z?), or the autopilot rejected GUIDED mode silently |
| `ACK = DENIED` on `SET_MODE GUIDED` | Vehicle isn't ArduCopter (or copter version is too old for `MAV_CMD_DO_SET_MODE`) |

## Extending it

`mission.py` is a state machine — add new states between TAKEOFF and
LAND for waypoints, payload triggers, sensor sweeps, etc. The
encoding helper `_make_command_long` matches the bridge's COMMAND_LONG
schema; new MAVLink commands need only their `MAV_CMD_*` integer and
the right `param1..param7` semantics from the
[MAV_CMD docs](https://mavlink.io/en/messages/common.html#mav_commands).

For higher-rate or non-COMMAND_LONG control (e.g.
`SET_POSITION_TARGET_LOCAL_NED`), the bridge crate would need a new
message added to `arrow_convert.rs` first; that's tracked separately.
