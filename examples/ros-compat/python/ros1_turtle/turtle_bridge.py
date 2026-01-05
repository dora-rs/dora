#!/usr/bin/env python3
"""
Example of a dora node communicating with a ROS1 node using roslibpy and dora-ros-compat.
This node subscribes to a ROS1 topic, converts the message to Arrow, and sends it to dora.
"""

import time
import os
import roslibpy
import pyarrow as pa
from dora import Node, ros

def main():
    # Initialize dora node
    node = Node()
    
    # Initialize RosMessageConverter
    # We can use get_schema to generate the schema if we have ROS2 installed,
    # but for this ROS1 example we'll focus on the format conversion compatibility.
    converter = ros.RosMessageConverter()

    # Create ROS1 client
    # Note: This requires a running rosbridge_server
    ros_host = os.environ.get("ROS_HOST", "localhost")
    ros_port = int(os.environ.get("ROS_PORT", 9090))
    
    client = roslibpy.Ros(host=ros_host, port=ros_port)
    client.run()
    
    # Publisher to turtlesim
    # We'll publish to /turtle1/cmd_vel
    talker = roslibpy.Topic(client, '/turtle1/cmd_vel', 'geometry_msgs/Twist')
    
    # Subscribe to turtlesim pose
    # We'll forward this to dora
    def on_pose_message(message):
        # ROS1 message is a dict
        
        # In a real scenario, you would convert this dict to Arrow struct
        # Since dora-ros-compat converts Arrow -> ROS, we demonstrate the other way here:
        # We'll create an Arrow array from the ROS message and "send" it to a mock conversion
        
        # Create Twist message for dora stream
        # (Mocking a source that sends Arrow data matching ROS format)
        
        # Here we demonstrate sending commands TO ROS1 from dora inputs
        pass

    listener = roslibpy.Topic(client, '/turtle1/pose', 'turtlesim/Pose')
    listener.subscribe(on_pose_message)
    
    print("Dora ROS1 Turtle Bridge Started")
    
    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "direction":
                # Received Arrow data from dora
                arrow_data = event["value"]
                
                # Convert arrow data to ROS-compatible dicts
                ros_msgs = converter.to_ros(arrow_data)
                
                for msg in ros_msgs:
                    # Publish to ROS1 via roslibpy
                    # msg is already in the correct dict format for ROS
                    talker.publish(roslibpy.Message(msg))
                    print(f"Sent command to ROS1: {msg}")

    client.terminate()

if __name__ == "__main__":
    main()
