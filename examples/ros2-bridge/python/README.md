# `ros2-bridge` Python examples

These examples show how to interact between ROS2 and dora from Python, using the
`dora` package's ROS2 bindings (`Ros2Context`, `Ros2Node`, …). They cover service
client/server, action client/server, parameters, and a turtlesim demo (topic
pub/sub).

See the [top-level README](../README.md) for the capability×language matrix and
[`docs/ros2-bridge.md`](../../../docs/ros2-bridge.md) for the full reference.

## Examples

| Directory        | `--example` name                      | Needs sourced ROS2 / peer   |
| ---------------- | ------------------------------------- | --------------------------- |
| `parameter`      | `python-ros2-dataflow-parameter`      | no (local parameter API)    |
| `service-client` | `python-ros2-dataflow-service-client` | yes (rclcpp service server) |
| `service-server` | `python-ros2-dataflow-service-server` | yes (rclcpp minimal client) |
| `action-client`  | `python-ros2-dataflow-action-client`  | yes (rclcpp action server)  |
| `action-server`  | `python-ros2-dataflow-action-server`  | no (dora server + client)¹  |
| `turtle`         | `python-ros2-dataflow`                | yes (`turtlesim`)           |

¹ A dora-hosted action server is not discoverable by a real `rcl` client
([ros2-client#4](https://github.com/jhelovuo/ros2-client/issues/4)), so the
`action-server` example pairs a dora server with a dora client. The Python
action client/server *do* support cancellation. See
[`docs/ros2-bridge.md`](../../../docs/ros2-bridge.md).

## Setup

```bash
uv venv --seed -p 3.12
source .venv/bin/activate
uv pip install -e ../../../apis/python/node   # workspace dora bindings
uv pip install pyarrow
```

Examples that talk to a real ROS2 node also need a sourced ROS2 install and the
matching example message package. The `parameter` example is self-contained
(local parameter API, no discovery) and needs no ROS2 install.

## Running

```bash
# self-contained
cargo run -p dora-ros2-bridge --example python-ros2-dataflow-parameter

# against a real ROS2 node (source ROS2 first)
source /opt/ros/humble/setup.bash
cargo run -p dora-ros2-bridge --example python-ros2-dataflow          # turtlesim
```
