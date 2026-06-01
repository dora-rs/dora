# `ros2-bridge` C++ examples

These examples show how to interact between ROS2 and dora from C++, using the
generated `cxx` bridge headers (`ros2-bridge/...`). They focus on the codegen
surfaces delivered for
[#1170](https://github.com/dora-rs/dora/issues/1170): service/action servers and
an action client, plus a turtlesim demo (topic pub/sub).

See the [top-level README](../README.md) for the capability×language matrix and
[`docs/ros2-bridge.md`](../../../docs/ros2-bridge.md) for the full reference.

## Examples

| Directory        | `--example` name                   | Needs sourced ROS2 / peer    |
| ---------------- | ---------------------------------- | ---------------------------- |
| `service-server` | `cxx-ros2-dataflow-service-server` | yes (rclcpp minimal client)  |
| `action-client`  | `cxx-ros2-dataflow-action-client`  | yes (rclcpp action server)¹  |
| `action-server`  | `cxx-ros2-dataflow-action-server`  | no (dora server + client)¹   |
| `turtle`         | `cxx-ros2-dataflow`                | yes (`turtlesim`)            |

All C++ examples are gated behind `--features ros2-examples`.

¹ A dora-hosted action server is not discoverable by a real `rcl` client
([ros2-client#4](https://github.com/jhelovuo/ros2-client/issues/4)), so the
`action-server` example pairs a dora C++ server with a dora C++ client; x86
nightly CI is the oracle (the deferred `get_result` round-trip stalls on the
arm64 dev harness). See [`docs/ros2-bridge.md`](../../../docs/ros2-bridge.md).

Service *client*, topic-only, and parameter examples are not provided in C++ —
those surfaces are covered by the Rust and Python examples (see the matrix).

## Setup

A sourced ROS2 installation plus a working C++ toolchain and CMake are required.
Some examples need an example message package, e.g.
`ros-humble-examples-rclcpp-minimal-client`.

## Running

```bash
source /opt/ros/humble/setup.bash
cargo run -p dora-ros2-bridge --example cxx-ros2-dataflow-service-server --features ros2-examples
cargo run -p dora-ros2-bridge --example cxx-ros2-dataflow-action-server --features ros2-examples
```
