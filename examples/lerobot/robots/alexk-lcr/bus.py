import enum

import pyarrow as pa

from typing import Union

from dynamixel_sdk import (
    PacketHandler,
    PortHandler,
    COMM_SUCCESS,
    GroupSyncRead,
    GroupSyncWrite,
)
from dynamixel_sdk import DXL_HIBYTE, DXL_HIWORD, DXL_LOBYTE, DXL_LOWORD

PROTOCOL_VERSION = 2.0
BAUD_RATE = 1_000_000
TIMEOUT_MS = 1000


def wrap_joints_and_values(
    joints: Union[list[str], pa.Array],
    values: Union[int, list[int], pa.Array],
) -> pa.StructArray:
    """
    Wraps joints and their corresponding values into a structured array.

    :param joints: A list, numpy array, or pyarrow array of joint names.
    :type joints: Union[list[str], np.array, pa.Array]
    :param values: A single integer value, or a list, numpy array, or pyarrow array of integer values.
                   If a single integer is provided, it will be broadcasted to all joints.
    :type values: Union[int, list[int], np.array, pa.Array]

    :return: A structured array with two fields:
             - "joints": A string field containing the names of the joints.
             - "values": An Int32Array containing the values corresponding to the joints.
    :rtype: pa.StructArray

    :raises ValueError: If lengths of joints and values do not match.

    Example:
    --------
    joints = ["shoulder_pan", "shoulder_lift", "elbow_flex"]
    values = [100, 200, 300]
    struct_array = wrap_joints_and_values(joints, values)

    This example wraps the given joints and their corresponding values into a structured array.

    Another example with a single integer value:
    joints = ["shoulder_pan", "shoulder_lift", "elbow_flex"]
    value = 150
    struct_array = wrap_joints_and_values(joints, value)

    This example broadcasts the single integer value to all joints and wraps them into a structured array.
    """

    if isinstance(values, int):
        values = [values] * len(joints)

    if len(joints) != len(values):
        raise ValueError("joints and values must have the same length")

    mask = pa.array([False] * len(values), type=pa.bool_())

    if isinstance(values, list):
        mask = pa.array([value is None for value in values])

    if isinstance(values, pa.Array):
        mask = values.is_null()

    return pa.StructArray.from_arrays(
        arrays=[joints, values], names=["joints", "values"], mask=mask
    ).drop_null()


class TorqueMode(enum.Enum):
    ENABLED = pa.scalar(1, pa.uint32())
    DISABLED = pa.scalar(0, pa.uint32())


class OperatingMode(enum.Enum):
    VELOCITY = pa.scalar(1, pa.uint32())
    POSITION = pa.scalar(3, pa.uint32())
    EXTENDED_POSITION = pa.scalar(4, pa.uint32())
    CURRENT_CONTROLLED_POSITION = pa.scalar(5, pa.uint32())
    PWM = pa.scalar(16, pa.uint32())


X_SERIES_CONTROL_TABLE = [
    ("Model_Number", 0, 2),
    ("Model_Information", 2, 4),
    ("Firmware_Version", 6, 1),
    ("ID", 7, 1),
    ("Baud_Rate", 8, 1),
    ("Return_Delay_Time", 9, 1),
    ("Drive_Mode", 10, 1),
    ("Operating_Mode", 11, 1),
    ("Secondary_ID", 12, 1),
    ("Protocol_Type", 13, 1),
    ("Homing_Offset", 20, 4),
    ("Moving_Threshold", 24, 4),
    ("Temperature_Limit", 31, 1),
    ("Max_Voltage_Limit", 32, 2),
    ("Min_Voltage_Limit", 34, 2),
    ("PWM_Limit", 36, 2),
    ("Current_Limit", 38, 2),
    ("Acceleration_Limit", 40, 4),
    ("Velocity_Limit", 44, 4),
    ("Max_Position_Limit", 48, 4),
    ("Min_Position_Limit", 52, 4),
    ("Shutdown", 63, 1),
    ("Torque_Enable", 64, 1),
    ("LED", 65, 1),
    ("Status_Return_Level", 68, 1),
    ("Registered_Instruction", 69, 1),
    ("Hardware_Error_Status", 70, 1),
    ("Velocity_I_Gain", 76, 2),
    ("Velocity_P_Gain", 78, 2),
    ("Position_D_Gain", 80, 2),
    ("Position_I_Gain", 82, 2),
    ("Position_P_Gain", 84, 2),
    ("Feedforward_2nd_Gain", 88, 2),
    ("Feedforward_1st_Gain", 90, 2),
    ("Bus_Watchdog", 98, 1),
    ("Goal_PWM", 100, 2),
    ("Goal_Current", 102, 2),
    ("Goal_Velocity", 104, 4),
    ("Profile_Acceleration", 108, 4),
    ("Profile_Velocity", 112, 4),
    ("Goal_Position", 116, 4),
    ("Realtime_Tick", 120, 2),
    ("Moving", 122, 1),
    ("Moving_Status", 123, 1),
    ("Present_PWM", 124, 2),
    ("Present_Current", 126, 2),
    ("Present_Velocity", 128, 4),
    ("Present_Position", 132, 4),
    ("Velocity_Trajectory", 136, 4),
    ("Position_Trajectory", 140, 4),
    ("Present_Input_Voltage", 144, 2),
    ("Present_Temperature", 146, 1),
]

MODEL_CONTROL_TABLE = {
    "x_series": X_SERIES_CONTROL_TABLE,
    "xl330-m077": X_SERIES_CONTROL_TABLE,
    "xl330-m288": X_SERIES_CONTROL_TABLE,
    "xl430-w250": X_SERIES_CONTROL_TABLE,
    "xm430-w350": X_SERIES_CONTROL_TABLE,
    "xm540-w270": X_SERIES_CONTROL_TABLE,
}


class DynamixelBus:

    def __init__(self, port: str, description: dict[str, (int, str)]):
        self.port = port
        self.descriptions = description
        self.motor_ctrl = {}

        for motor_name, (motor_id, motor_model) in description.items():
            if motor_model not in MODEL_CONTROL_TABLE:
                raise ValueError(f"Model {motor_model} is not supported.")

            self.motor_ctrl[motor_name] = {}

            self.motor_ctrl[motor_name]["id"] = motor_id
            for data_name, address, bytes_size in MODEL_CONTROL_TABLE[motor_model]:
                self.motor_ctrl[motor_name][data_name] = {
                    "addr": address,
                    "bytes_size": bytes_size,
                }

        self.port_handler = PortHandler(self.port)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            raise OSError(f"Failed to open port {self.port}")

        self.port_handler.setBaudRate(BAUD_RATE)
        self.port_handler.setPacketTimeoutMillis(TIMEOUT_MS)

        self.group_readers = {}
        self.group_writers = {}

    def close(self):
        self.port_handler.closePort()

    def write(self, data_name: str, data: pa.StructArray):
        motor_ids = [
            self.motor_ctrl[motor_name.as_py()]["id"]
            for motor_name in data.field("joints")
        ]

        values = pa.Array.from_buffers(
            pa.uint32(),
            length=len(data.field("values")),
            buffers=data.field("values").buffers(),
        )

        group_key = f"{data_name}_" + "_".join([str(idx) for idx in motor_ids])

        first_motor_name = list(self.motor_ctrl.keys())[0]

        packet_address = self.motor_ctrl[first_motor_name][data_name]["addr"]
        packet_bytes_size = self.motor_ctrl[first_motor_name][data_name]["bytes_size"]

        init_group = data_name not in self.group_readers

        if init_group:
            self.group_writers[group_key] = GroupSyncWrite(
                self.port_handler,
                self.packet_handler,
                packet_address,
                packet_bytes_size,
            )

        for idx, value in zip(motor_ids, values):
            value = value.as_py()
            if value is None:
                continue

            if packet_bytes_size == 1:
                data = [
                    DXL_LOBYTE(DXL_LOWORD(value)),
                ]
            elif packet_bytes_size == 2:
                data = [
                    DXL_LOBYTE(DXL_LOWORD(value)),
                    DXL_HIBYTE(DXL_LOWORD(value)),
                ]
            elif packet_bytes_size == 4:
                data = [
                    DXL_LOBYTE(DXL_LOWORD(value)),
                    DXL_HIBYTE(DXL_LOWORD(value)),
                    DXL_LOBYTE(DXL_HIWORD(value)),
                    DXL_HIBYTE(DXL_HIWORD(value)),
                ]
            else:
                raise NotImplementedError(
                    f"Value of the number of bytes to be sent is expected to be in [1, 2, 4], but {packet_bytes_size} "
                    f"is provided instead."
                )

            if init_group:
                self.group_writers[group_key].addParam(idx, data)
            else:
                self.group_writers[group_key].changeParam(idx, data)

        comm = self.group_writers[group_key].txPacket()
        if comm != COMM_SUCCESS:
            raise ConnectionError(
                f"Write failed due to communication error on port {self.port} for group_key {group_key}: "
                f"{self.packet_handler.getTxRxResult(comm)}"
            )

    def read(self, data_name: str, motor_names: pa.Array) -> pa.StructArray:
        motor_ids = [
            self.motor_ctrl[motor_name.as_py()]["id"] for motor_name in motor_names
        ]

        group_key = f"{data_name}_" + "_".join([str(idx) for idx in motor_ids])

        first_motor_name = list(self.motor_ctrl.keys())[0]

        packet_address = self.motor_ctrl[first_motor_name][data_name]["addr"]
        packet_bytes_size = self.motor_ctrl[first_motor_name][data_name]["bytes_size"]

        if data_name not in self.group_readers:
            self.group_readers[group_key] = GroupSyncRead(
                self.port_handler,
                self.packet_handler,
                packet_address,
                packet_bytes_size,
            )

            for idx in motor_ids:
                self.group_readers[group_key].addParam(idx)

        comm = self.group_readers[group_key].txRxPacket()
        if comm != COMM_SUCCESS:
            raise ConnectionError(
                f"Read failed due to communication error on port {self.port} for group_key {group_key}: "
                f"{self.packet_handler.getTxRxResult(comm)}"
            )

        values = pa.array(
            [
                self.group_readers[group_key].getData(
                    idx, packet_address, packet_bytes_size
                )
                for idx in motor_ids
            ],
            type=pa.uint32(),
        )
        values = values.from_buffers(pa.int32(), len(values), values.buffers())

        return wrap_joints_and_values(motor_names, values)

    def write_torque_enable(self, torque_mode: pa.StructArray):
        self.write("Torque_Enable", torque_mode)

    def write_operating_mode(self, operating_mode: pa.StructArray):
        self.write("Operating_Mode", operating_mode)

    def read_position(self, motor_names: pa.Array) -> pa.StructArray:
        return self.read("Present_Position", motor_names)

    def read_velocity(self, motor_names: pa.Array) -> pa.StructArray:
        return self.read("Present_Velocity", motor_names)

    def read_current(self, motor_names: pa.Array) -> pa.StructArray:
        return self.read("Present_Current", motor_names)

    def write_goal_position(self, goal_position: pa.StructArray):
        self.write("Goal_Position", goal_position)

    def write_goal_current(self, goal_current: pa.StructArray):
        self.write("Goal_Current", goal_current)

    def write_position_p_gain(self, position_p_gain: pa.StructArray):
        self.write("Position_P_Gain", position_p_gain)

    def write_position_i_gain(self, position_i_gain: pa.StructArray):
        self.write("Position_I_Gain", position_i_gain)

    def write_position_d_gain(self, position_d_gain: pa.StructArray):
        self.write("Position_D_Gain", position_d_gain)
