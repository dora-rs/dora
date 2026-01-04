#!/usr/bin/env python
"""Example node demonstrating ROS message format conversion."""

from dora import Node, ros

def main():
    """Main function demonstrating ROS message conversion."""
    node = Node()
    converter = ros.RosMessageConverter()
    
    print("ROS Message Converter Example")
    print("Waiting for twist data...")
    
    for event in node:
        if event is None:
            break
            
        event_type = event["type"]
        
        if event_type == "INPUT":
            event_id = event["id"]
            
            if event_id == "twist_data":
                try:
                    # Convert Arrow array to ROS message
                    # Returns a list of messages (one for each element in the batch)
                    ros_msgs = converter.to_ros(event["value"], "geometry_msgs/Twist")
                    
                    for ros_twist in ros_msgs:
                        # Extract and display the ROS message data
                        linear = ros_twist["linear"]
                        angular = ros_twist["angular"]
                        
                        print(f"\nReceived ROS Twist message:")
                        print(f"  Linear:  x={linear['x']:.2f}, y={linear['y']:.2f}, z={linear['z']:.2f}")
                        print(f"  Angular: x={angular['x']:.2f}, y={angular['y']:.2f}, z={angular['z']:.2f}")
                        
                    # Send output (optional)
                    node.send_output("converted_twist", event["value"])
                    
                except Exception as e:
                    print(f"Error converting message: {e}")

if __name__ == "__main__":
    main()
