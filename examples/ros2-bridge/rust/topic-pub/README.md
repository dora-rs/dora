# `rust-ros2-topic-pub-dataflow` Example

This example shows how to publish from Dora to a ROS2 topic. The dataflow consists of nodes in [ROS2 `examples_rclcpp_minimal_subscriber`](https://index.ros.org/p/examples_rclcpp_minimal_subscriber).

## Setup

This examples requires a sourced ROS2 installation.

- To set up ROS2, follow the [ROS2 installation](https://docs.ros.org/en/iron/Installation.html) guide.
- Don't forget to [source the ROS2 setup files](https://docs.ros.org/en/iron/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#source-the-setup-files)
- Follow tasks 1 and 2 of the [ROS2 turtlesim tutorial](https://docs.ros.org/en/iron/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html#id3)
  - Install the `examples_rclcpp_minimal_subscriber` package
  - Start the ROS2 subscriber node through `ros2 run examples_rclcpp_minimal_subscriber subscriber_lambda`(arbitrary nodes in this packages is available)
  - Build the Dora node through `dora build dataflow.yml`
  - Start the Dora dataflow through `dora run dataflow.yml`

## Running

After sourcing the ROS2 installation and starting both the `examples_rclcpp_minimal_subscriber` package, you can run this example to publish messages to ROS2's node:

```
cargo run --example rust-ros2-topic-pub-dataflow --features ros2-examples
```
