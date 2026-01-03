# ROS Message Format Compatibility

A small library that converts Dora's Arrow data format to ROS message dictionaries in Python.

## Overview

Sometimes you need to work with ROS message structures in Dora nodes, but you don't need a full ROS bridge. This library lets you convert Arrow arrays to Python dictionaries that match ROS message formats.

Right now it only supports `geometry_msgs/Twist`, but it's easy to extend for other message types.

## Usage

```python
from dora import Node
from dora.ros import RosMessageConverter

node = Node()
converter = RosMessageConverter()

for event in node:
    if event["type"] == "INPUT":
        ros_twist = converter.to_ros(event["value"], "geometry_msgs/Twist")
        
        linear = ros_twist["linear"]
        angular = ros_twist["angular"]
        print(f"Linear: x={linear['x']}, y={linear['y']}, z={linear['z']}")
```

## Example

Check out `examples/ros-compat/python/twist_converter/` for a working example.

## Limitations

- Only supports `geometry_msgs/Twist` for now
- One-way conversion: Arrow → ROS dict (not the other way around)
- Doesn't connect to ROS/ROS2 - it's just format conversion

For actual ROS integration (publishing/subscribing), use `dora-ros2-bridge` instead.

## Future

We can add more message types as needed. The structure is simple enough to extend.