from dora import Node
import pyarrow as pa
import pygame

class LogitechF710:
    """Logitech F710 controller mapping"""
    fullName = 'Logitech Wireless Gamepad F710'

    def __init__(self, joystickNumber=0):
        self.joystickNumber = joystickNumber
        self.axisNames = {
            0: 'LEFT-X',    # Left stick X axis
            1: 'LEFT-Y',    # Left stick Y axis
            2: 'LT',        # Left trigger
            3: 'RIGHT-X',   # Right stick X axis
            4: 'RIGHT-Y',   # Right stick Y axis
            5: 'RT',        # Right trigger
            6: 'DPAD-X',    # D-pad X axis
            7: 'DPAD-Y'     # D-pad Y axis
        }
        self.buttonNames = {
            0: 'A',
            1: 'B',
            2: 'X',
            3: 'Y',
            4: 'LB',
            5: 'RB',
            6: 'BACK',
            7: 'START',
            8: 'LOGITECH',
            9: 'LEFT-STICK',
            10: 'RIGHT-STICK'
        }

def main():
    node = Node("gamepad")
    
    # Initialize pygame for joystick handling
    pygame.init()
    pygame.joystick.init()
    
    # Check for connected joysticks
    if pygame.joystick.get_count() == 0:
        print("No gamepad found! Please connect your Logitech F710 controller.")
        return
        
    # Initialize the first joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    
    # Control parameters
    max_linear_speed = 2.0  # Maximum speed in m/s
    max_angular_speed = 1.5  # Maximum angular speed in rad/s
    
    print("Gamepad Controls:")
    print("Left Stick Y-Axis: Forward/Backward")
    print("Left Stick X-Axis: Left/Right Turn")
    print("Mode switch should be in 'D' position")
    print("Press Ctrl+C to exit")

    try:
        for event in node:
            # Process pygame events
            pygame.event.pump()
            # print("#############################################")
            try:
                # Get axis values (-1 to 1)
                forward = -joystick.get_axis(1)  # Left stick Y (inverted)
                turn = -joystick.get_axis(0)     # Left stick X
                
                # Apply deadzone
                deadzone = 0.05
                forward = 0.0 if abs(forward) < deadzone else forward
                turn = 0.0 if abs(turn) < deadzone else turn
                
                # Calculate velocities
                forward_speed = forward * max_linear_speed
                turn_speed = turn * max_angular_speed
                
                # Create cmd_vel array
                cmd_vel = [
                    forward_speed,  # Linear X
                    0.0,           # Linear Y
                    0.0,           # Linear Z
                    0.0,           # Angular X
                    0.0,           # Angular Y
                    turn_speed     # Angular Z
                ]
                
                # Send command
                node.send_output(
                    output_id="cmd_vel",
                    data=pa.array(cmd_vel, type=pa.float64()),
                    metadata={"type": "cmd_vel"}
                )
                # print("cmd vel : ", cmd_vel)
                
            except Exception as e:
                print(f"Controller error: {e}")
                
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        # Cleanup
        pygame.quit()
        # Send zero velocity
        zero_cmd = [0.0] * 6
        try:
            node.send_output(
                output_id="cmd_vel",
                data=pa.array(zero_cmd, type=pa.float64()),
                metadata={"type": "cmd_vel"}
            )
        except Exception as e:
            print(f"Failed to send zero velocity: {e}")

if __name__ == "__main__":
    main()
