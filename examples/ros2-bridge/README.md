# `ros2-bridge` examples

These examples show how to bridge between ROS2 and dora. dora speaks the ROS2
wire protocol directly through either the pure-Rust DDS stack (`ros2-client` +
`rustdds`) or native Zenoh compatible with `rmw_zenoh_cpp`. It never links
`rcl`/`rclcpp`/`rclpy`, so a sourced ROS2 install is only needed for
the examples that talk to a *real* ROS2 node (e.g. `turtlesim`, or an `rclcpp`
peer).

For the full feature reference (architecture, the two bridge surfaces, and the
known limitations) see [`docs/ros2-bridge.md`](../../docs/ros2-bridge.md), also
published in the guide under *Advanced → ROS2 bridge*.

## Two surfaces

1. **Native code APIs** — generated Rust/C++ types (from the ROS2 message
   definitions) plus the Python bindings. You write a dora node that constructs
   a `ros2-client` node and publishes/subscribes/calls services/actions. The
   per-language subdirectories below demonstrate this surface.
2. **YAML / dynamic bridge** — no codegen: a daemon-spawnable bridge node mapping
   ROS2 topics/services/actions to dora inputs/outputs via the dataflow YAML. See
   the `yaml-bridge*` directories.

## Capability × language matrix (native APIs)

| Capability      | Rust                       | Python              | C++              |
| --------------- | -------------------------- | ------------------- | ---------------- |
| topic pub/sub   | `topic-pub`, `topic-sub`   | (in `turtle`)       | (in `turtle`)    |
| service client  | `service-client`           | `service-client`    | —                |
| service server  | `service-server`           | `service-server`    | `service-server` |
| action client   | `action-client`            | `action-client`     | `action-client`  |
| action server   | `action-server`            | `action-server`     | `action-server`  |
| parameters      | `parameter`                | `parameter`         | —                |
| turtlesim demo  | `turtle`                   | `turtle`            | `turtle`         |

Empty cells are intentional, not missing work: each protocol surface is
demonstrated in at least one language, and the C++ examples focus on the
codegen surfaces delivered for [#1170](https://github.com/dora-rs/dora/issues/1170)
(service/action servers + an action client). Topic pub/sub and parameters are
covered by the Rust and Python examples.

## Running

DDS remains the default. YAML bridges select native Zenoh explicitly:

```yaml
ros2:
  transport:
    kind: zenoh
    compatibility: humble # or rep2016
    config_uri: /etc/zenoh/dora.json5 # optional
  topic: /chatter
  message_type: std_msgs/String
  direction: subscribe
```

Start `rmw_zenohd` for the selected ROS distribution and use the same
`ROS_DOMAIN_ID` on Dora and ROS peers. See `dataflow-zenoh.yml` in each YAML
example directory and `scripts/ros2-zenoh-interop.sh` for pinned real-peer tests.

Each example is wired as a Cargo `[[example]]` of the `dora-ros2-bridge` crate
(its `run.rs` builds the dataflow and launches it, spawning any ROS2 peer it
needs). Run them by name:

```bash
# self-contained (no ROS2 install needed): pure-Rust DDS only
cargo run -p dora-ros2-bridge --example rust-ros2-dataflow-parameter

# talks to a real ROS2 node / rclcpp peer: source ROS2 first
source /opt/ros/humble/setup.bash
cargo run -p dora-ros2-bridge --example rust-ros2-dataflow            # turtlesim
cargo run -p dora-ros2-bridge --example python-ros2-dataflow-service-server
```

Examples that require a sourced ROS2 install and an example message package are
gated behind the `ros2-examples` feature, e.g.:

```bash
cargo run -p dora-ros2-bridge --example cxx-ros2-dataflow-action-server --features ros2-examples
```

See each language subdirectory's `README.md` for the per-example commands and
prerequisites.

## Platform notes

Some examples have platform caveats (full detail in
[`docs/ros2-bridge.md`](../../docs/ros2-bridge.md)):

- A dora-hosted **action** server is not discoverable by a real
  `rcl`/`rclcpp`/`rclpy` client ([ros2-client#4](https://github.com/jhelovuo/ros2-client/issues/4)),
  so the action examples pair a dora server with a dora client in one dataflow.
  **Service** servers are discovered fine by a real `ros2` client.
- The deferred action `get_result` round-trip is flaky in upstream
  `ros2-client`/`rustdds` (it repeatedly hung the x86 nightly job and stalls on
  the macOS/arm64 dev harness). The Rust/C++ action examples are therefore
  **not run in nightly CI**; validate them with `scripts/ros2dev.sh qa` on x86
  Linux before a release. Tracked in
  [#1170](https://github.com/dora-rs/dora/issues/1170).

The `parameter` examples use the *local* parameter API (no discovery), so they
run deterministically on every platform.
