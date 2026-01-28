#!/usr/bin/env python3
"""
Dora node that generates Twist commands as Arrow data.
"""

from dora import Node
import pyarrow as pa
import time
import math

def main():
    node = Node()
    
    # Define Twist schema
    linear_fields = [
        pa.field("x", pa.float64()),
        pa.field("y", pa.float64()),
        pa.field("z", pa.float64()),
    ]
    angular_fields = [
        pa.field("x", pa.float64()),
        pa.field("y", pa.float64()),
        pa.field("z", pa.float64()),
    ]
    
    twist_fields = [
        pa.field("linear", pa.struct(linear_fields)),
        pa.field("angular", pa.struct(angular_fields)),
    ]
    
    twist_schema = pa.struct(twist_fields)
    
    print("Turtle Control Node Started")
    
    start_time = time.time()
    
    while True:
        elapsed = time.time() - start_time
        
        # Generate a circle pattern
        linear_x = 2.0
        angular_z = 1.0
        
        # Create Twist message data
        twist_data = [{
            "linear": {"x": linear_x, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": angular_z}
        }]
        
        # Create Arrow array
        twist_array = pa.array(twist_data, type=twist_schema)
        
        # Send to bridge
        node.send_output("direction", twist_array)
        
        time.sleep(0.1)

if __name__ == "__main__":
    main()
