# `rust-ros2-dataflow` Example

This example shows how to publish/subscribe to both ROS2 and Dora. The dataflow consists of a single node that sends random movement commands to the [ROS2 `turtlesim_node`](https://docs.ros.org/en/iron/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html).

## Setup

This examples requires a sourced ROS2 installation.

- To set up ROS2, follow the [ROS2 installation](https://docs.ros.org/en/iron/Installation.html) guide.
- Don't forget to [source the ROS2 setup files](https://docs.ros.org/en/iron/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#source-the-setup-files)
- Follow tasks 1 and 2 of the [ROS2 turtlesim tutorial](https://docs.ros.org/en/iron/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html#id3)
  - Install the turtlesim package
  - Install the examples-rclcpp-minimal-service package 
## Running

> [!NOTE]
> Please replace the `<ros-distro>` with the actual ROS2 distribution you are using.

```bash
source /opt/ros/<ros-distro>/setup.bash
cargo run --package dora-ros2-bridge --example rust-ros2-dataflow
```
