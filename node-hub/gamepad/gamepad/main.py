"""Gamepad controller node for Dora.

This module provides a Dora node that reads input from a controller and publishes:
1. velocity commands for robot control
2. raw controller state for debugging and custom mappings
"""

from dora import Node
import pygame
import pyarrow as pa
import json

class Controller:
    """controller mapping."""

    def __init__(self):
        """Change this according to your controller mapping. Currently Logitech F710."""
        self.axisNames = {
            'LEFT-X': 0,
            'LEFT-Y': 1,        
            'RIGHT-X': 2,
            'RIGHT-Y': 3,   
        }
        self.buttonNames = {
            'X': 0,
            'A': 1,
            'B': 2,
            'Y': 3,
            'LB': 4,
            'RB': 5,
            'LT': 6,
            'RT': 7,
            'BACK': 8,
            'START': 9,
            'LEFT-STICK': 10,
            'RIGHT-STICK': 11
        }

def main():
    node = Node("gamepad")
    
    pygame.init()
    pygame.joystick.init()
    
    if pygame.joystick.get_count() == 0:
        print("No gamepad found! Please connect your controller.")
        return
        
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    controller = Controller()
    
    max_linear_speed = 1.0  # Maximum speed in m/s
    max_angular_speed = 1.5  # Maximum angular speed in rad/s
    
    print(f"Detected controller: {joystick.get_name()}")
    print(f"Number of axes: {joystick.get_numaxes()}")
    print(f"Number of buttons: {joystick.get_numbuttons()}")
    print("Press Ctrl+C to exit")

    try:
        for event in node:
            pygame.event.pump()

            # Get all controller states
            axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
            buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]

            # Create raw control state
            raw_control = {
                "axes": axes,
                "buttons": buttons,
                "mapping": {
                    "axes": controller.axisNames,
                    "buttons": controller.buttonNames
                }
            }
            # print("raw_control:", raw_control)  # uncomment for debugging and re-map

            # Regular cmd_vel processing
            forward = -joystick.get_axis(controller.axisNames['LEFT-Y'])
            turn = -joystick.get_axis(controller.axisNames['LEFT-X'])
            
            deadzone = 0.05
            forward = 0.0 if abs(forward) < deadzone else forward
            turn = 0.0 if abs(turn) < deadzone else turn
            
            forward_speed = forward * max_linear_speed
            turn_speed = turn * max_angular_speed
        
            cmd_vel = [forward_speed, 0.0, 0.0, 0.0, 0.0, turn_speed]
            
            node.send_output(
                output_id="cmd_vel",
                data=pa.array(cmd_vel, type=pa.float64()),
                metadata={"type": "cmd_vel"}
            )

            node.send_output(
                output_id="raw_control",
                data=pa.array([json.dumps(raw_control)], type=pa.string()),
                metadata={"type": "raw_control"}
            )

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        pygame.quit()
        # Send zero velocity at cleanup
        zero_cmd = [0.0] * 6
        node.send_output(
            output_id="cmd_vel",
            data=pa.array(zero_cmd, type=pa.float64()),
            metadata={"type": "cmd_vel"}
        )

if __name__ == "__main__":
    main()
