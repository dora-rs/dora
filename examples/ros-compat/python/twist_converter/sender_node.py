#!/usr/bin/env python
"""Creates and sends a Twist message as Arrow data."""

from dora import Node
import pyarrow as pa

def main():
    node = Node()
    
    # Create a Twist message: linear velocity (1.0, 0.0, 0.0) and angular (0.0, 0.0, 0.5)
    linear_data = [{"x": 1.0, "y": 0.0, "z": 0.0}]
    angular_data = [{"x": 0.0, "y": 0.0, "z": 0.5}]
    
    # Define the structure
    linear_fields = [
        pa.field("x", pa.float64(), False),
        pa.field("y", pa.float64(), False),
        pa.field("z", pa.float64(), False),
    ]
    angular_fields = [
        pa.field("x", pa.float64(), False),
        pa.field("y", pa.float64(), False),
        pa.field("z", pa.float64(), False),
    ]
    
    # Create arrays
    linear_array = pa.array(linear_data, type=pa.struct(linear_fields))
    angular_array = pa.array(angular_data, type=pa.struct(angular_fields))
    
    # Create the Twist struct
    twist_fields = [
        pa.field("linear", linear_array.type, False),
        pa.field("angular", angular_array.type, False),
    ]
    twist_data = [{"linear": linear_data[0], "angular": angular_data[0]}]
    twist_array = pa.array(twist_data, type=pa.struct(twist_fields))
    
    print("Sending Twist message...")
    node.send_output("twist_data", twist_array)
    
    # Give it a moment before exiting
    import time
    time.sleep(0.1)

if __name__ == "__main__":
    main()