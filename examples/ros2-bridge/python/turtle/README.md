# Quick Python ROS2 example

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
