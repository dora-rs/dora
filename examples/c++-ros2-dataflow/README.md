# `cxx-ros2-dataflow` Example

This c++ example shows how to publish/subscribe to both ROS2 and Dora. The dataflow consists of a single node that sends random movement commands to the [ROS2 `turtlesim_node`](https://docs.ros.org/en/iron/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html).

## Setup

This examples requires a sourced ROS2 installation.

- To set up ROS2, follow the [ROS2 installation](https://docs.ros.org/en/iron/Installation.html) guide.
- Don't forget to [source the ROS2 setup files](https://docs.ros.org/en/iron/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#source-the-setup-files)
- Follow tasks 1 and 2 of the [ROS2 turtlesim tutorial](https://docs.ros.org/en/iron/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html#id3)
  - Install the turtlesim package
  - Start the turtlesim node through `ros2 run turtlesim turtlesim_node`

## Running


From terminal 1 , sourcing the ROS2 installation and start ROS2 turtlesim window
```
source /opt/ros/galactic/setup.bash
ros2 run turtlesim turtlesim_node
```

From terminal 2 from dora folder 
```
cargo run --example cxx-ros2-dataflow --features ros2-examples
```
And you will see the turtle move a few steps.
