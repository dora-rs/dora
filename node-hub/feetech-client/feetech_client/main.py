"""Feetech Client: This node is used to represent a chain of feetech motors. It can be used to read positions, velocities, currents, and set goal positions and currents."""

import argparse
import json
import os

import numpy as np
import pyarrow as pa
from dora import Node

from .bus import FeetechBus, TorqueMode, wrap_joints_and_values


class Client:
    """TODO: Add docstring."""

    def __init__(self, config: dict[str, any]):
        """TODO: Add docstring."""
        self.config = config

        description = {}
        for i in range(len(config["ids"])):
            description[config["joints"][i]] = (config["ids"][i], config["models"][i])

        self.config["joints"] = pa.array(config["joints"], pa.string())
        self.bus = FeetechBus(config["port"], description)

        # Set client configuration values and raise errors if the values are not set to indicate that the motors are not
        # configured correctly

        self.bus.write_torque_enable(self.config["torque"])

        self.node = Node(config["name"])

    def run(self):
        """TODO: Add docstring."""
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
                elif event_id == "end":
                    break

            elif event_type == "ERROR":
                raise ValueError("An error occurred in the dataflow: " + event["error"])

    def close(self):
        """TODO: Add docstring."""
        self.bus.write_torque_enable(
            wrap_joints_and_values(
                self.config["joints"],
                [TorqueMode.DISABLED.value] * len(self.config["joints"]),
            ),
        )

    def pull_position(self, node, metadata):
        """TODO: Add docstring."""
        try:
            struct = self.bus.read_position(self.config["joints"])
            metadata["encoding"] = "jointstate"
            node.send_output(
                "position",
                pa.array(
                    np.deg2rad(
                        ((struct.flatten()[1].to_numpy() - 2048) / 4096.0) * 360
                    ),
                    type=pa.float32(),
                ),
                metadata,
            )

        except ConnectionError as e:
            print("Error reading position:", e)

    def pull_velocity(self, node, metadata):
        """TODO: Add docstring."""
        try:
            node.send_output(
                "velocity",
                self.bus.read_velocity(self.config["joints"]),
                metadata,
            )
        except ConnectionError as e:
            print("Error reading velocity:", e)

    def pull_current(self, node, metadata):
        """TODO: Add docstring."""
        try:
            node.send_output(
                "current",
                self.bus.read_current(self.config["joints"]),
                metadata,
            )
        except ConnectionError as e:
            print("Error reading current:", e)

    def write_goal_position(self, goal_position: pa.StructArray):
        """TODO: Add docstring."""
        try:
            self.bus.write_goal_position(goal_position)
        except ConnectionError as e:
            print("Error writing goal position:", e)


def main():
    """Handle dynamic nodes, ask for the name of the node in the dataflow."""
    parser = argparse.ArgumentParser(
        description="Feetech Client: This node is used to represent a chain of feetech motors. "
        "It can be used to read "
        "positions, velocities, currents, and set goal positions and currents.",
    )

    parser.add_argument(
        "--name",
        type=str,
        required=False,
        help="The name of the node in the dataflow.",
        default="feetech_client",
    )
    parser.add_argument(
        "--port",
        type=str,
        required=False,
        help="The port of the feetech motors.",
        default=None,
    )
    parser.add_argument(
        "--config",
        type=str,
        help="The configuration of the feetech motors.",
        default=None,
    )

    args = parser.parse_args()

    # Check if port is set
    if not os.environ.get("PORT") and args.port is None:
        raise ValueError(
            "The port is not set. Please set the port of the feetech motors in the environment variables or as an "
            "argument.",
        )

    port = os.environ.get("PORT") if args.port is None else args.port

    # Check if config is set
    if not os.environ.get("CONFIG") and args.config is None:
        raise ValueError(
            "The configuration is not set. Please set the configuration of the feetech motors in the environment "
            "variables or as an argument.",
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
                [(TorqueMode.DISABLED.value) for joint in joints],
                type=pa.uint32(),
            ),
        ),
    }

    client = Client(bus)
    client.run()
    client.close()


if __name__ == "__main__":
    main()
