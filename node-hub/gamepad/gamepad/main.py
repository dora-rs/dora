"""Gamepad controller node for Dora.

This module provides a Dora node that reads input from a controller and publishes velocity commands for robot control.
It handles controller mapping, deadzone filtering, and velocity scaling.
"""

from dora import Node
import pygame
import pyarrow as pa


class Controller:
    """controller mapping."""

    def __init__(self):
        """Change this according to your controller mapping. Currently Logitech F710."""
        self.axisNames = {
            "LEFT-X": 0,
            "LEFT-Y": 1,
            "LT": 2,
            "RIGHT-X": 3,
            "RIGHT-Y": 4,
            "RT": 5,
            "DPAD-X": 6,
            "DPAD-Y": 7,
        }
        self.buttonNames = {
            "A": 0,
            "B": 1,
            "X": 2,
            "Y": 3,
            "LB": 4,
            "RB": 5,
            "BACK": 6,
            "START": 7,
            "LOGITECH": 8,
            "LEFT-STICK": 9,
            "RIGHT-STICK": 10,
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

    print("Gamepad Controls:")
    print("Left Stick Y-Axis: Forward/Backward")
    print("Left Stick X-Axis: Left/Right Turn")
    print("Mode switch should be in 'D' position")
    print("Press Ctrl+C to exit")

    try:
        for event in node:
            pygame.event.pump()

            forward = -joystick.get_axis(controller.axisNames["LEFT-Y"])
            turn = -joystick.get_axis(controller.axisNames["LEFT-X"])

            deadzone = 0.05
            forward = 0.0 if abs(forward) < deadzone else forward
            turn = 0.0 if abs(turn) < deadzone else turn

            forward_speed = forward * max_linear_speed
            turn_speed = turn * max_angular_speed

            cmd_vel = [forward_speed, 0.0, 0.0, 0.0, 0.0, turn_speed]

            node.send_output(
                output_id="cmd_vel",
                data=pa.array(cmd_vel, type=pa.float64()),
                metadata={"type": "cmd_vel"},
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
            metadata={"type": "cmd_vel"},
        )


if __name__ == "__main__":
    main()
