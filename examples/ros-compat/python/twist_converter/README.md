# ROS Message Format Conversion Example

This example shows how to convert Dora Arrow data to ROS message format.

## What it does

The example has two nodes:
1. `sender_node.py` - Creates some Arrow data that represents a Twist message
2. `twist_node.py` - Converts that Arrow data to a ROS message dictionary

## Running

```bash
cd examples/ros-compat/python/twist_converter
dora build dataflow.yml
dora run dataflow.yml
```

Or:

```bash
cargo run --example python-ros-compat-twist
```

## The code

```python
from dora import Node, ros

node = Node()
converter = ros.RosMessageConverter()

for event in node:
    if event["type"] == "INPUT":
        # Returns a list of dicts (one per message in the batch)
        ros_msgs = converter.to_ros(event["value"], "geometry_msgs/Twist")
        
        for msg in ros_msgs:
            linear = msg["linear"]
            print(f"Linear: x={linear['x']}, y={linear['y']}, z={linear['z']}")
```

## Note

This is just format conversion - it doesn't actually connect to ROS. If you need to publish/subscribe to ROS topics, use `dora-ros2-bridge`.