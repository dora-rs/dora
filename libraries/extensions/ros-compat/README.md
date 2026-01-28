# ROS Message Format Compatibility

A small library that converts Dora's Arrow data format to ROS message dictionaries in Python.

## Overview

Sometimes you need to work with ROS message structures in Dora nodes, but you don't need a full ROS bridge. This library lets you convert Arrow arrays to Python dictionaries that match ROS message formats.

It supports **any** message type that is structured in the Arrow schema.

## Features

- **Generic Message Support**: Automatically converts any Arrow data structure to ROS message format
- **Automatic Schema Generation**: `get_schema()` method generates PyArrow schemas from ROS message definitions (requires ROS2)
- **No Manual Type Definitions**: Uses PyArrow's built-in conversion capabilities
- **Batch Processing**: Handles batched Arrow data efficiently

## Usage

### Basic Conversion

```python
from dora import Node, ros

node = Node()
converter = ros.RosMessageConverter()

for event in node:
    if event["type"] == "INPUT":
        # Returns a list of dicts (one per message in the batch)
        ros_msgs = converter.to_ros(event["value"])
        
        for msg in ros_msgs:
            linear = msg["linear"]
            print(f"Linear: x={linear['x']}, y={linear['y']}, z={linear['z']}")
```

### Automatic Schema Generation

```python
from dora import ros

# Requires ROS2 installation and AMENT_PREFIX_PATH environment variable
converter = ros.RosMessageConverter()

# Automatically generate PyArrow schema for any ROS message type
schema = converter.get_schema("geometry_msgs/Twist")
print(schema)  # pyarrow.Schema with correct field types

# Works with any ROS message type
pose_schema = converter.get_schema("geometry_msgs/Pose")
image_schema = converter.get_schema("sensor_msgs/Image")
```

## Example

Check out `examples/ros-compat/python/twist_converter/` for a working example.

## Limitations

- **Python-only**: Currently no Rust API (contributions welcome!)
- **Format conversion only**: This is not a full ROS2 API replacement
- **Schema generation requires ROS2**: The `get_schema()` method needs ROS2 installed and `AMENT_PREFIX_PATH` set
- One-way conversion: Arrow → ROS dict (not the other way around)
- Doesn't connect to ROS/ROS2 - it's just format conversion

For actual ROS integration (publishing/subscribing), use `dora-ros2-bridge` instead.