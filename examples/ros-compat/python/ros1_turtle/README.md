# ROS1 Turtle Sim Compatibility Example

This example demonstrates how to use `dora-ros-compat` to interface a dora dataflow with a ROS1 system using `rosbridge_suite` and `roslibpy`.

## Prerequisites

1. **ROS Noetic** (or Melodic) installed
2. `rosbridge_suite` installed:
   ```bash
   sudo apt-get install ros-noetic-rosbridge-suite
   sudo apt-get install ros-noetic-turtlesim
   ```
3. Python dependencies:
   ```bash
   pip install roslibpy pyarrow dora-rs
   ```

## How it works

1. **control_node**: Generates movement commands (Twist) as generic Arrow structs. It has NO knowledge of ROS.
2. **turtle_bridge**:
   - Subscribes to the Arrow data from `control_node`.
   - Uses `ros.RosMessageConverter` to convert the Arrow structs to ROS-compatible Python dictionaries.
   - Publishes these dictionaries to ROS1 via `roslibpy`.

## Running the Example

1. Start ROS Core, Rosbridge, and Turtlesim:
   ```bash
   # Term 1
   roscore
   
   # Term 2
   roslaunch rosbridge_server rosbridge_websocket.launch
   
   # Term 3
   rosrun turtlesim turtlesim_node
   ```

2. Run the dora dataflow:
   ```bash
   dora start dataflow.yml
   ```

The turtle in the simulator should start moving in a circle.

## Key Feature Demonstrated

The `turtle_bridge.py` uses `converter.to_ros(arrow_data)` which automatically handles the conversion from the Arrow format (used by dora) to the dictionary format required by `roslibpy`, making integration seamlessly easy.
