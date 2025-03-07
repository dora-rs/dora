"""
Dynamixel Client: This node is used to represent a chain of dynamixel motors. It can be used to read positions,
velocities, currents, and set goal positions and currents.
"""

import os
import time
import argparse
import json

import pyarrow as pa

from dora import Node

from .bus import DynamixelBus, TorqueMode, wrap_joints_and_values


class Client:

    def __init__(self, config: dict[str, any]):
        self.config = config

        description = {}
        for i in range(len(config["ids"])):
            description[config["joints"][i]] = (config["ids"][i], config["models"][i])

        self.config["joints"] = pa.array(config["joints"], pa.string())
        self.bus = DynamixelBus(config["port"], description)

        # Set client configuration values, raise errors if the values are not set to indicate that the motors are not
        # configured correctly

        self.bus.write_torque_enable(self.config["torque"])
        self.bus.write_goal_current(self.config["goal_current"])

        time.sleep(0.1)
        self.bus.write_position_d_gain(self.config["D"])

        time.sleep(0.1)
        self.bus.write_position_i_gain(self.config["I"])

        time.sleep(0.1)
        self.bus.write_position_p_gain(self.config["P"])

        self.node = Node(config["name"])

    def run(self):
        for event in self.node:
            event_type = event["type"]

            if event_type == "INPUT":
                event_id = event["id"]

                if event_id == "pull_position":
                    self.pull_position(self.node, event["metadata"])
                elif event_id == "pull_velocity":
                    self.pull_velocity(self.node, event["metadata"])
                elif event_id == "pull_current":
                    self.pull_current(self.node, event["metadata"])
                elif event_id == "write_goal_position":
                    self.write_goal_position(event["value"])
                elif event_id == "write_goal_current":
                    self.write_goal_current(event["value"])
                elif event_id == "end":
                    break

            elif event_type == "ERROR":
                raise ValueError("An error occurred in the dataflow: " + event["error"])

    def close(self):
        self.bus.write_torque_enable(
            wrap_joints_and_values(
                self.config["joints"],
                [TorqueMode.DISABLED.value] * len(self.config["joints"]),
            )
        )

    def pull_position(self, node, metadata):
        try:
            node.send_output(
                "position",
                self.bus.read_position(self.config["joints"]),
                metadata,
            )

        except ConnectionError as e:
            print("Error reading position:", e)

    def pull_velocity(self, node, metadata):
        try:
            node.send_output(
                "velocity",
                self.bus.read_velocity(self.config["joints"]),
                metadata,
            )
        except ConnectionError as e:
            print("Error reading velocity:", e)

    def pull_current(self, node, metadata):
        try:
            node.send_output(
                "current",
                self.bus.read_current(self.config["joints"]),
                metadata,
            )
        except ConnectionError as e:
            print("Error reading current:", e)

    def write_goal_position(self, goal_position: pa.StructArray):
        try:
            self.bus.write_goal_position(goal_position)
        except ConnectionError as e:
            print("Error writing goal position:", e)

    def write_goal_current(self, goal_current: pa.StructArray):
        try:
            self.bus.write_goal_current(goal_current)
        except ConnectionError as e:
            print("Error writing goal current:", e)


def main():
    # Handle dynamic nodes, ask for the name of the node in the dataflow
    parser = argparse.ArgumentParser(
        description="Dynamixel Client: This node is used to represent a chain of dynamixel motors. It can be used to "
        "read positions, velocities, currents, and set goal positions and currents."
    )

    parser.add_argument(
        "--name",
        type=str,
        required=False,
        help="The name of the node in the dataflow.",
        default="dynamixel_client",
    )
    parser.add_argument(
        "--port",
        type=str,
        required=False,
        help="The port of the dynamixel motors.",
        default=None,
    )
    parser.add_argument(
        "--config",
        type=str,
        help="The configuration of the dynamixel motors.",
        default=None,
    )

    args = parser.parse_args()

    # Check if port is set
    if not os.environ.get("PORT") and args.port is None:
        raise ValueError(
            "The port is not set. Please set the port of the dynamixel motors in the environment variables or as an "
            "argument."
        )

    port = os.environ.get("PORT") if args.port is None else args.port

    # Check if config is set
    if not os.environ.get("CONFIG") and args.config is None:
        raise ValueError(
            "The configuration is not set. Please set the configuration of the dynamixel motors in the environment "
            "variables or as an argument."
        )

    with open(os.environ.get("CONFIG") if args.config is None else args.config) as file:
        config = json.load(file)

    joints = config.keys()

    # Create configuration
    bus = {
        "name": args.name,
        "port": port,  # (e.g. "/dev/ttyUSB0", "COM3")
        "ids": [config[joint]["id"] for joint in joints],
        "joints": list(config.keys()),
        "models": [config[joint]["model"] for joint in joints],
        "torque": wrap_joints_and_values(
            pa.array(config.keys(), pa.string()),
            pa.array(
                [
                    (
                        TorqueMode.ENABLED.value
                        if config[joint]["torque"]
                        else TorqueMode.DISABLED.value
                    )
                    for joint in joints
                ],
                type=pa.uint32(),
            ),
        ),
        "goal_current": wrap_joints_and_values(
            pa.array(config.keys(), pa.string()),
            pa.array(
                [config[joint]["goal_current"] for joint in joints], type=pa.uint32()
            ),
        ),
        "P": wrap_joints_and_values(
            pa.array(config.keys(), pa.string()),
            pa.array([config[joint]["P"] for joint in joints], type=pa.uint32()),
        ),
        "I": wrap_joints_and_values(
            pa.array(config.keys(), pa.string()),
            pa.array([config[joint]["I"] for joint in joints], type=pa.uint32()),
        ),
        "D": wrap_joints_and_values(
            pa.array(config.keys(), pa.string()),
            pa.array([config[joint]["D"] for joint in joints], type=pa.uint32()),
        ),
    }

    print("Dynamixel Client Configuration: ", bus, flush=True)

    client = Client(bus)
    client.run()
    client.close()


if __name__ == "__main__":
    main()
