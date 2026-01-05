#!/usr/bin/env python3
"""
Unit test for test_mock_bridge.py
Mocks roslibpy to verify that the bridge correctly converts Arrow data
and attempts to publish to ROS.
"""

import unittest
from unittest.mock import MagicMock, patch
import sys

# Mock roslibpy module BEFORE importing it
mock_roslibpy = MagicMock()
sys.modules["roslibpy"] = mock_roslibpy

import pyarrow as pa
from dora import ros

class TestRos1Bridge(unittest.TestCase):
    def setUp(self):
        self.converter = ros.RosMessageConverter()

    def test_bridge_logic(self):
        # Setup Mocks
        mock_message = mock_roslibpy.Message
        mock_topic = mock_roslibpy.Topic
        mock_talker = mock_topic.return_value
        
        # Create Twist schema (same as control_node)
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

        # Create sample Arrow data (simulating input from dora)
        twist_data = [{
            "linear": {"x": 2.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 1.0}
        }]
        arrow_array = pa.array(twist_data, type=twist_schema)

        # --- Simulate Bridge Logic ---
        
        # 1. Convert Arrow to ROS dict
        ros_msgs = self.converter.to_ros(arrow_array)
        
        # Verify conversion correctness
        expected_msg = {'linear': {'x': 2.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 1.0}}
        self.assertEqual(ros_msgs[0], expected_msg)
        
        # 2. Simulate Publishing
        for msg in ros_msgs:
            # Mock the wrapping into roslibpy.Message
            ros_message_obj = mock_message(msg)
            
            # Publish
            mock_talker.publish(ros_message_obj)
            
        # Verify calls
        mock_message.assert_called_with(expected_msg)
        mock_talker.publish.assert_called()
        
        print("✓ Mock ROS1 Bridge Logic Verified")

if __name__ == '__main__':
    unittest.main()
