# Quick Python ROS2 example

- Install the turtlesim package
- On Ubuntu, you can install it with `sudo apt install ros-<ros-distro>-turtlesim`

> [!NOTE]
> Please replace the `<ros-distro>` with the actual ROS2 distribution you are using.

To get started:

```bash
source /opt/ros/humble/setup.bash && ros2 run turtlesim turtlesim_node
cargo run --example python-ros2-dataflow --features="ros2-examples"
```

- alternatively:

```bash
source /opt/ros/humble/setup.bash && ros2 run turtlesim turtlesim_node

# cd examples/python-ros2-dataflow
uv pip install -e ../.../apis/python/node --reinstall
dora build dataflow.yml --uv
dora run dataflow.yml --uv
```
