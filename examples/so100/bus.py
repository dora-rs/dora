import enum

import pyarrow as pa

from typing import Union

from scservo_sdk import (
    PacketHandler,
    PortHandler,
    COMM_SUCCESS,
    GroupSyncRead,
    GroupSyncWrite,
)
from scservo_sdk import SCS_HIBYTE, SCS_HIWORD, SCS_LOBYTE, SCS_LOWORD

PROTOCOL_VERSION = 0
BAUD_RATE = 1_000_000
TIMEOUT_MS = 1000


def wrap_joints_and_values(
    joints: Union[list[str], pa.Array],
    values: Union[list[int], pa.Array],
) -> pa.StructArray:
    return pa.StructArray.from_arrays(
        arrays=[joints, values],
        names=["joints", "values"],
    )


class TorqueMode(enum.Enum):
    ENABLED = pa.scalar(1, pa.uint32())
    DISABLED = pa.scalar(0, pa.uint32())


class OperatingMode(enum.Enum):
    ONE_TURN = pa.scalar(0, pa.uint32())


SCS_SERIES_CONTROL_TABLE = [
    ("Model", 3, 2),
    ("ID", 5, 1),
    ("Baud_Rate", 6, 1),
    ("Return_Delay", 7, 1),
    ("Response_Status_Level", 8, 1),
    ("Min_Angle_Limit", 9, 2),
    ("Max_Angle_Limit", 11, 2),
    ("Max_Temperature_Limit", 13, 1),
    ("Max_Voltage_Limit", 14, 1),
    ("Min_Voltage_Limit", 15, 1),
    ("Max_Torque_Limit", 16, 2),
    ("Phase", 18, 1),
    ("Unloading_Condition", 19, 1),
    ("LED_Alarm_Condition", 20, 1),
    ("P_Coefficient", 21, 1),
    ("D_Coefficient", 22, 1),
    ("I_Coefficient", 23, 1),
    ("Minimum_Startup_Force", 24, 2),
    ("CW_Dead_Zone", 26, 1),
    ("CCW_Dead_Zone", 27, 1),
    ("Protection_Current", 28, 2),
    ("Angular_Resolution", 30, 1),
    ("Offset", 31, 2),
    ("Mode", 33, 1),
    ("Protective_Torque", 34, 1),
    ("Protection_Time", 35, 1),
    ("Overload_Torque", 36, 1),
    ("Speed_closed_loop_P_proportional_coefficient", 37, 1),
    ("Over_Current_Protection_Time", 38, 1),
    ("Velocity_closed_loop_I_integral_coefficient", 39, 1),
    ("Torque_Enable", 40, 1),
    ("Acceleration", 41, 1),
    ("Goal_Position", 42, 2),
    ("Goal_Time", 44, 2),
    ("Goal_Speed", 46, 2),
    ("Lock", 55, 1),
    ("Present_Position", 56, 2),
    ("Present_Speed", 58, 2),
    ("Present_Load", 60, 2),
    ("Present_Voltage", 62, 1),
    ("Present_Temperature", 63, 1),
    ("Status", 65, 1),
    ("Moving", 66, 1),
    ("Present_Current", 69, 2),
]

MODEL_CONTROL_TABLE = {
    "scs_series": SCS_SERIES_CONTROL_TABLE,
    "sts3215": SCS_SERIES_CONTROL_TABLE,
}


class FeetechBus:

    def __init__(self, port: str, description: dict[str, (np.uint8, str)]):
        """
        Args:
            port: the serial port to connect to the Feetech bus
            description: a dictionary containing the description of the motors connected to the bus. The keys are the
            motor names and the values are tuples containing the motor id and the motor model.
        """

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

        values = [
            np.uint32(32767 - value.as_py()) if value < 0 else np.uint32(value.as_py())
            for value in data.field("values")
        ]

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
            if value is None:
                continue

            if packet_bytes_size == 1:
                data = [
                    SCS_LOBYTE(SCS_LOWORD(value)),
                ]
            elif packet_bytes_size == 2:
                data = [
                    SCS_LOBYTE(SCS_LOWORD(value)),
                    SCS_HIBYTE(SCS_LOWORD(value)),
                ]
            elif packet_bytes_size == 4:
                data = [
                    SCS_LOBYTE(SCS_LOWORD(value)),
                    SCS_HIBYTE(SCS_LOWORD(value)),
                    SCS_LOBYTE(SCS_HIWORD(value)),
                    SCS_HIBYTE(SCS_HIWORD(value)),
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

        values = pa.array(
            [
                value.as_py() if value.as_py() < 32767 else 32767 - value.as_py()
                for value in values
            ],
            type=pa.int32(),
        )

        return wrap_joints_and_values(motor_names, values)

    def write_torque_enable(self, torque_mode: pa.StructArray):
        self.write("Torque_Enable", torque_mode)

    def write_operating_mode(self, operating_mode: pa.StructArray):
        self.write("Mode", operating_mode)

    def read_position(self, motor_names: pa.Array) -> pa.StructArray:
        return self.read("Present_Position", motor_names)

    def read_velocity(self, motor_names: pa.Array) -> pa.StructArray:
        return self.read("Present_Velocity", motor_names)

    def read_current(self, motor_names: pa.Array) -> pa.StructArray:
        return self.read("Present_Current", motor_names)

    def write_goal_position(self, goal_position: pa.StructArray):
        self.write("Goal_Position", goal_position)

    def write_max_angle_limit(self, max_angle_limit: pa.StructArray):
        self.write("Max_Angle_Limit", max_angle_limit)

    def write_min_angle_limit(self, min_angle_limit: pa.StructArray):
        self.write("Min_Angle_Limit", min_angle_limit)
