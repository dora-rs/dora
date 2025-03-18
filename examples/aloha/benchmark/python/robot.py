"""Module for controlling robot hardware and movements.

This module provides functionality for controlling robot hardware components,
including servo motors and various control modes.
"""

import time
from enum import Enum, auto
from typing import Union

import numpy as np
from dynamixel import OperatingMode, ReadAttribute
from dynamixel_sdk import (
    DXL_HIBYTE,
    DXL_HIWORD,
    DXL_LOBYTE,
    DXL_LOWORD,
    GroupSyncRead,
    GroupSyncWrite,
)


class MotorControlType(Enum):
    """Enumeration of different motor control modes.
    
    Defines the various control modes available for the robot's motors,
    including PWM control, position control, and disabled states.
    """

    PWM = auto()
    POSITION_CONTROL = auto()
    DISABLED = auto()
    UNKNOWN = auto()


class Robot:
    """A class for controlling robot hardware components.
    
    This class provides methods for controlling robot servos, managing different
    control modes, and reading sensor data.
    """

    # def __init__(self, device_name: str, baudrate=1_000_000, servo_ids=[1, 2, 3, 4, 5]):
    def __init__(self, dynamixel, baudrate=1_000_000, servo_ids=[1, 2, 3, 4, 5]):
        """Initialize the robot controller.

        Args:
            dynamixel: Dynamixel controller instance
            baudrate: Communication baudrate for servo control
            servo_ids: List of servo IDs to control

        """
        self.servo_ids = servo_ids
        self.dynamixel = dynamixel
        # self.dynamixel = Dynamixel.Config(baudrate=baudrate, device_name=device_name).instantiate()
        self.position_reader = GroupSyncRead(
            self.dynamixel.portHandler,
            self.dynamixel.packetHandler,
            ReadAttribute.POSITION.value,
            4,
        )
        for id in self.servo_ids:
            self.position_reader.addParam(id)

        self.velocity_reader = GroupSyncRead(
            self.dynamixel.portHandler,
            self.dynamixel.packetHandler,
            ReadAttribute.VELOCITY.value,
            4,
        )
        for id in self.servo_ids:
            self.velocity_reader.addParam(id)

        self.pos_writer = GroupSyncWrite(
            self.dynamixel.portHandler,
            self.dynamixel.packetHandler,
            self.dynamixel.ADDR_GOAL_POSITION,
            4,
        )
        for id in self.servo_ids:
            self.pos_writer.addParam(id, [2048])

        self.pwm_writer = GroupSyncWrite(
            self.dynamixel.portHandler,
            self.dynamixel.packetHandler,
            self.dynamixel.ADDR_GOAL_PWM,
            2,
        )
        for id in self.servo_ids:
            self.pwm_writer.addParam(id, [2048])
        self._disable_torque()
        self.motor_control_state = MotorControlType.DISABLED

    def read_position(self, tries=2):
        """Read the current joint positions of the robot.

        Args:
            tries: Maximum number of attempts to read positions

        Returns:
            list: Joint positions in range [0, 4096]

        """
        result = self.position_reader.txRxPacket()
        if result != 0:
            if tries > 0:
                return self.read_position(tries=tries - 1)
            print("failed to read position!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        positions = []
        for id in self.servo_ids:
            position = self.position_reader.getData(id, ReadAttribute.POSITION.value, 4)
            if position > 2**31:
                position -= 2**32
            positions.append(position)
        return positions

    def read_velocity(self):
        """Read the current joint velocities of the robot.

        Returns:
            list: Current joint velocities

        """
        self.velocity_reader.txRxPacket()
        velocties = []
        for id in self.servo_ids:
            velocity = self.velocity_reader.getData(id, ReadAttribute.VELOCITY.value, 4)
            if velocity > 2**31:
                velocity -= 2**32
            velocties.append(velocity)
        return velocties

    def set_goal_pos(self, action):
        """Set goal positions for the robot joints.

        Args:
            action: List or numpy array of target joint positions in range [0, 4096]

        """
        if self.motor_control_state is not MotorControlType.POSITION_CONTROL:
            self._set_position_control()
        for i, motor_id in enumerate(self.servo_ids):
            data_write = [
                DXL_LOBYTE(DXL_LOWORD(action[i])),
                DXL_HIBYTE(DXL_LOWORD(action[i])),
                DXL_LOBYTE(DXL_HIWORD(action[i])),
                DXL_HIBYTE(DXL_HIWORD(action[i])),
            ]
            self.pos_writer.changeParam(motor_id, data_write)

        self.pos_writer.txPacket()

    def set_pwm(self, action):
        """Set PWM values for the servos.

        Args:
            action: List or numpy array of PWM values in range [0, 885]

        """
        if self.motor_control_state is not MotorControlType.PWM:
            self._set_pwm_control()
        for i, motor_id in enumerate(self.servo_ids):
            data_write = [
                DXL_LOBYTE(DXL_LOWORD(action[i])),
                DXL_HIBYTE(DXL_LOWORD(action[i])),
            ]
            self.pwm_writer.changeParam(motor_id, data_write)

        self.pwm_writer.txPacket()

    def set_trigger_torque(self):
        """Set a constant torque for the last servo in the chain.
        
        This is useful for the trigger of the leader arm.
        """
        self.dynamixel._enable_torque(self.servo_ids[-1])
        self.dynamixel.set_pwm_value(self.servo_ids[-1], 200)

    def limit_pwm(self, limit: Union[int, list, np.ndarray]):
        """Limit the PWM values for the servos in position control.

        Args:
            limit: PWM limit value in range 0-885

        """
        if isinstance(limit, int):
            limits = [
                limit,
            ] * 5
        else:
            limits = limit
        self._disable_torque()
        for motor_id, limit in zip(self.servo_ids, limits):
            self.dynamixel.set_pwm_limit(motor_id, limit)
        self._enable_torque()

    def _disable_torque(self):
        print(f"disabling torque for servos {self.servo_ids}")
        for motor_id in self.servo_ids:
            self.dynamixel._disable_torque(motor_id)

    def _enable_torque(self):
        print(f"enabling torque for servos {self.servo_ids}")
        for motor_id in self.servo_ids:
            self.dynamixel._enable_torque(motor_id)

    def _set_pwm_control(self):
        self._disable_torque()
        for motor_id in self.servo_ids:
            self.dynamixel.set_operating_mode(motor_id, OperatingMode.PWM)
        self._enable_torque()
        self.motor_control_state = MotorControlType.PWM

    def _set_position_control(self):
        self._disable_torque()
        for motor_id in self.servo_ids:
            self.dynamixel.set_operating_mode(motor_id, OperatingMode.POSITION)
        self._enable_torque()
        self.motor_control_state = MotorControlType.POSITION_CONTROL


if __name__ == "__main__":
    robot = Robot(device_name="/dev/tty.usbmodem57380045631")
    robot._disable_torque()
    for _ in range(10000):
        s = time.time()
        pos = robot.read_position()
        elapsed = time.time() - s
        print(f"read took {elapsed} pos {pos}")
