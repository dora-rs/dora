# Quick Python ROS2 example

To get started:

```bash
source /opt/ros/humble/setup.bash && ros2 run turtlesim turtlesim_node &
source /opt/ros/humble/setup.bash && ros2 run examples_rclcpp_minimal_service service_main &
cargo run --example python-ros2-dataflow --features="ros2-examples"
```

- alternatively:

```bash
source /opt/ros/humble/setup.bash && ros2 run turtlesim turtlesim_node &
source /opt/ros/humble/setup.bash && ros2 run examples_rclcpp_minimal_service service_main &

# cd examples/python-ros2-dataflow
uv pip install -e ../.../apis/python/node
dora build dataflow.yml --uv
dora run dataflow.yml --uv
```
