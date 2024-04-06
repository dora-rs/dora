# `cxx-ros2-dataflow` Example

This c++ example shows how to publish/subscribe to both ROS2 and Dora. The dataflow consists of a single node that sends random movement commands to the [ROS2 `turtlesim_node`](https://docs.ros.org/en/iron/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html).

## Setup

This examples requires a sourced ROS2 installation.

- To set up ROS2, follow the [ROS2 installation](https://docs.ros.org/en/iron/Installation.html) guide.
- Don't forget to [source the ROS2 setup files](https://docs.ros.org/en/iron/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#source-the-setup-files)
- Follow tasks 1 and 2 of the [ROS2 turtlesim tutorial](https://docs.ros.org/en/iron/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html#id3)
  - Install the turtlesim package
  - Start the turtlesim node through `ros2 run turtlesim turtlesim_node`

## Running pub/sub example

A ROS2 client to pubish turtlesim ROS2 messages and a DORA node can subscribe and visualize it.

From terminal 1 , sourcing the ROS2 installation and start ROS2 turtlesim window
```
source /opt/ros/galactic/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run turtlesim turtlesim_node
```

From terminal 2 from dora folder. Note the source command here is necessary as this allow ROS2 message types to be found and compile dynamically.
```
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/galactic/setup.bash
cargo run --example cxx-ros2-dataflow --features ros2-examples
```
And you will see the turtle move a few steps.

## Running service example
The current service code example is a service client. To test with service server we can test with either ROS2 demo or ros2-client
- if using ROS2 demo the the command line is:
```
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run demo_nodes_cpp add_two_ints_server
```

start DORA service client from another terminal
```
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
cargo run --example cxx-ros2-dataflow --features ros2-examples
```

- if using ros2-client the command line is:
```
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
cargo run --example=ros2_service_server
```

then start DORA service client from another terminal
```
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
cargo run --example cxx-ros2-dataflow --features ros2-examples
```

You can also put export RMW_IMPLEMENTATION=rmw_fastrtps_cpp into .bashrc

