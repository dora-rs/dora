"""Gamepad controller node for Dora."""

from dora import Node
import pygame
import pyarrow as pa
import json

class Controller:
    """Controller mapping."""

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
            'RIGHT-STICK': 11,
        }
        self.hatIndex = 0  # Index of the D-pad hat

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
    
    move_speed = 0.05  # Fixed increment for D-pad
    max_linear_z_speed = 0.1  # Maximum linear Z speed
    max_angular_speed = 0.8  # Maximum angular speed
    
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
            
            # Get hat state (D-pad)
            dpad_x, dpad_y = 0, 0
            if joystick.get_numhats() > 0:
                dpad_x, dpad_y = joystick.get_hat(controller.hatIndex)

            # Create raw control state
            raw_control = {
                "axes": axes,
                "buttons": buttons,
                "hats": [[dpad_x, dpad_y]],
                "mapping": {
                    "axes": controller.axisNames,
                    "buttons": controller.buttonNames
                }
            }

            # cmd_vel processing:
            # 1. D-pad vertical for X axis
            # 2. D-pad horizontal for Y axis
            # 3. Right stick vertical for Z axis
            # 4. Right stick horizontal for rotation around Z
            # 5. Left stick vertical for rotation around X
            # 6. Left stick horizontal for rotation around Y

            deadzone = 0.05

            # Linear X velocity from D-pad vertical
            linear_x = 0.0
            if dpad_y != 0:
                linear_x = dpad_y * move_speed

            # Linear Y velocity from D-pad horizontal
            linear_y = 0.0
            if dpad_x != 0:
                linear_y = dpad_x * move_speed
            
            # Linear Z velocity from right stick vertical
            right_y = -joystick.get_axis(controller.axisNames['RIGHT-Y'])
            right_y = 0.0 if abs(right_y) < deadzone else right_y
            linear_z = right_y * max_linear_z_speed
            
            # Angular Z velocity (rotation) from right stick horizontal
            right_x = -joystick.get_axis(controller.axisNames['RIGHT-X'])
            right_x = 0.0 if abs(right_x) < deadzone else right_x
            angular_z = 0 * max_angular_speed # TODO: Make z non zero, but on my gamepad the value is never zero
            
            # Angular X velocity from left stick vertical
            left_y = -joystick.get_axis(controller.axisNames['LEFT-Y'])
            left_y = 0.0 if abs(left_y) < deadzone else left_y
            angular_x = left_y * max_angular_speed
            
            # Angular Y velocity from left stick horizontal
            left_x = -joystick.get_axis(controller.axisNames['LEFT-X'])
            left_x = 0.0 if abs(left_x) < deadzone else left_x
            angular_y = left_x * max_angular_speed
            
            cmd_vel = [linear_x, linear_y, linear_z, angular_x, angular_y, angular_z]
            if any(cmd_vel):
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
        zero_cmd = [0.0] * 6
        node.send_output(
            output_id="cmd_vel",
            data=pa.array(zero_cmd, type=pa.float64()),
            metadata={"type": "cmd_vel"}
        )

if __name__ == "__main__":
    main()