# `cxx-ros2-dataflow` Example

This c++ example shows how to publish/subscribe to both ROS2 and Dora. The dataflow consists of a single node that sends random movement commands to the [ROS2 `turtlesim_node`](https://docs.ros.org/en/iron/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html).

## Setup

This examples requires a sourced ROS2 installation.

- To set up ROS2, follow the [ROS2 installation](https://docs.ros.org/en/iron/Installation.html) guide.
- Don't forget to [source the ROS2 setup files](https://docs.ros.org/en/iron/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#source-the-setup-files)
- Follow tasks 1 and 2 of the [ROS2 turtlesim tutorial](https://docs.ros.org/en/iron/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html#id3)
  - Install the turtlesim package
  - Install the examples-rclcpp-minimal-service package 
  - On Ubuntu, you can install them with `sudo apt install ros-<ros-distro>-examples-rclcpp-minimal-service ros-<ros-distro>-turtlesim`
  
## Run example

> [!NOTE]
> Please replace the `<ros-distro>` with the actual ROS2 distribution you are using.

```bash
source /opt/ros/<ros-distro>/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
cargo run --package dora-ros2-bridge --example cxx-ros2-dataflow
```

## Alternative (manual)
Not recommended, because manually building c++ node is a heavy job.

### Build the example
Please refer to the `run.rs` file in the same directory with this `README.md` file.

### Run

1. From terminal 1, source ROS2 and start ROS2 turtlesim window
```bash
source /opt/ros/<ros-distro>/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run turtlesim turtlesim_node
```

2. From terminal 2, source ROS2 and start the add two ints service node.
```bash
source /opt/ros/<ros-distro>/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run examples_rclcpp_minimal_service service_main
```

3. From terminal 3 in the folder of dora repository. Note the source command here is necessary as this allows ROS2 message types to be found and compile dynamically.
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/<ros-distro>/setup.bash
dora run dataflow.yml
```

You can also put export RMW_IMPLEMENTATION=rmw_fastrtps_cpp into .bashrc
