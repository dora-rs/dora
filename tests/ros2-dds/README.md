# ROS 2 DDS interoperability

`scripts/ros2-dds-interop.sh <humble|kilted> [<rmw|all>] [<case|all>]` runs
dora against real `rclpy` peers over Fast-DDS (`rmw_fastrtps_cpp`) and
CycloneDDS (`rmw_cyclonedds_cpp`). It covers two dora surfaces:

- the in-process **Python** bridge (`Ros2Context`), through
  `tests/ros2-zenoh/peers/dora_peer.py --transport dds`;
- the **YAML bridge** (`dora-ros2-bridge-node`), through the
  `examples/ros2-bridge/yaml-bridge-*` dataflows.

The `rclpy` peers are the RMW-agnostic ones in `tests/ros2-zenoh/peers/`.

## How it works

The ROS side runs in a container from `docker-compose.ros2-dds.yml`. It uses
the same digest-pinned images as the Zenoh harness, host networking (so DDS
multicast discovery reaches the host), and has CycloneDDS installed. The
installed RMW versions are printed at the start of each run.

The dora side is built on the host for the selected distro: the Python wheel
and the YAML-bridge binaries are built with `ros2-<distro>`, into
`target/ros2-dds-<distro>/`, because Kilted uses the 16-byte Gid.

Every run uses `ROS_DOMAIN_ID` 23 (override with `DORA_ROS2_DDS_DOMAIN`), so
other DDS traffic on the host stays out. Logs go to
`target/ros2-dds-logs/<project>/<rmw>/`.

Requirements: docker with the compose plugin, `maturin`, Python 3.11 or newer.

## Results

Each cell prints one line, `<RESULT> <distro> <rmw> <case>`:

| Result | Meaning | Job |
|---|---|---|
| `PASS` | passed | ok |
| `FAIL` | failed, and the cell is not a known failure | fails |
| `XFAIL` | failed, and the cell is listed in `expected-failures.txt` | ok |
| `XPASS` | passed, but the cell is listed as a known failure | fails |

## Known failures

`expected-failures.txt` lists cells that fail today, one per line:

```
<distro|*> <rmw|*> <case> #<issue>  # reason
```

Rules:

- Every entry names an open issue.
- An unknown distro, RMW or case name is rejected (exit 2), so a typo cannot
  silently stop matching.
- When a fix lands, its cells report `XPASS` and the job fails until the
  entry is removed in the same change.

`scripts/ros2-dds-interop.sh --self-test` checks this classification without
docker.
