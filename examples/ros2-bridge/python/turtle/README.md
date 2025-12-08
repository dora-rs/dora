# Quick Python ROS2 example

> [!NOTE]
> Please replace the `<ros-distro>` with the actual ROS2 distribution you are using.

To get started:

```bash
source /opt/ros/<ros-distro>/setup.bash
cargo run --package dora-ros2-bridge --example python-ros2-dataflow
```

- alternatively:

```bash
# you need to run the turtlesim manually
source /opt/ros/<ros-distro>/setup.bash && ros2 run turtlesim turtlesim_node

# cd examples/python-ros2-dataflow
uv pip install -e ../../../../apis/python/node --reinstall
dora build dataflow.yml --uv
dora run dataflow.yml --uv
```
