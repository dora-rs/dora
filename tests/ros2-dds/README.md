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
<distro|*> <rmw|*> <case> #<issue> [flaky]  # reason
```

Rules:

- Every entry names an open issue.
- `flaky` marks a timing-dependent bug, such as a discovery race. Such a cell
  may pass or fail, and either result keeps the job green; the result line
  still names the issue. Mark every cell of a timing race `flaky`, including
  cells that failed in every local run: a few runs on one machine cannot tell
  "always fails" from "usually fails", and on a runner such a cell may pass
  and fail the job with `XPASS`. Use it only for a race, not for a
  deterministic bug: a flaky entry hides that cell's later failures for other
  reasons.
- An unknown distro, RMW or case name is rejected (exit 2), so a typo cannot
  silently stop matching.
- When a fix lands, its plain entries report `XPASS` and the job fails until
  they are removed in the same change. Flaky entries never XPASS, so remove
  them by hand together with the fix.

`scripts/ros2-dds-interop.sh --self-test` checks this classification without
docker.

## Running against other artifacts

`DORA_ROS2_DDS_PREBUILT=<dir>` skips building the bridge. The run then uses
`<dir>/python` (the installed wheel and pyarrow) and
`<dir>/bin/dora-ros2-bridge-node`. Use it to check that the matrix catches a
known bug in an older build: with ros2-client 0.10.1 (#4206), the CycloneDDS
service/action cells fail.
