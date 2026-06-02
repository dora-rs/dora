# `ros2-bridge` Rust examples

These examples show how to interact between ROS2 and dora from Rust, using the
generated message types and a `ros2-client` node. They cover topic pub/sub,
service client/server, action client/server, and parameters.

See the [top-level README](../README.md) for the capability×language matrix and
[`docs/ros2-bridge.md`](../../../docs/ros2-bridge.md) for the full reference.

## Examples

| Directory        | `--example` name                    | Needs sourced ROS2 / peer    |
| ---------------- | ----------------------------------- | ---------------------------- |
| `parameter`      | `rust-ros2-dataflow-parameter`      | no (local parameter API)     |
| `topic-pub`      | `rust-ros2-dataflow-topic-pub`      | yes (publishes to ROS2)      |
| `topic-sub`      | `rust-ros2-dataflow-topic-sub`      | yes (subscribes from ROS2)   |
| `service-client` | `rust-ros2-dataflow-service-client` | yes (rclcpp service server)  |
| `service-server` | `rust-ros2-dataflow-service-server` | yes (rclcpp minimal client)  |
| `action-client`  | `rust-ros2-dataflow-action-client`  | yes (rclcpp action server)¹  |
| `action-server`  | `rust-ros2-dataflow-action-server`  | no (dora server + client)¹   |
| `turtle`         | `rust-ros2-dataflow`                | yes (`turtlesim`)            |

¹ Action examples are gated behind `--features ros2-examples`. A dora-hosted
action server is not discoverable by a real `rcl` client
([ros2-client#4](https://github.com/jhelovuo/ros2-client/issues/4)), so the
`action-server` example pairs a dora server with a dora client. The deferred
`get_result` round-trip is flaky in upstream `ros2-client`/`rustdds` (it
repeatedly hung the x86 nightly job and stalls on the arm64 dev harness), so the
action examples are **not run in nightly CI** — validate them with
`scripts/ros2dev.sh qa` on x86 Linux before a release ([#1170](https://github.com/dora-rs/dora/issues/1170)).
See [`docs/ros2-bridge.md`](../../../docs/ros2-bridge.md).

## Setup

The examples that talk to a real ROS2 node require a sourced ROS2 installation
and the matching example message package:

- [Install ROS2](https://docs.ros.org/en/iron/Installation.html) and
  [source the setup files](https://docs.ros.org/en/iron/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#source-the-setup-files).
- Install the corresponding ROS2 examples package (e.g.
  `ros-humble-examples-rclcpp-minimal-client`).

The `parameter` example is self-contained (pure-Rust DDS, local parameter API)
and needs no ROS2 install.

## Running

```bash
# self-contained
cargo run -p dora-ros2-bridge --example rust-ros2-dataflow-parameter

# against a real ROS2 node (source ROS2 first)
source /opt/ros/humble/setup.bash
cargo run -p dora-ros2-bridge --example rust-ros2-dataflow            # turtlesim

# action examples need the ros2-examples feature
cargo run -p dora-ros2-bridge --example rust-ros2-dataflow-action-server --features ros2-examples
```
