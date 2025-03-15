import os
import argparse
import json

from dora import Node

from pwm_position_control.load import load_control_table_from_json_conversion_tables
from pwm_position_control.transform import logical_to_pwm_with_offset_arrow


def main():
    parser = argparse.ArgumentParser(
        description="Interpolation LCR Node: This Dora node is used to calculates appropriate goal positions for the "
        "LCR followers knowing a Leader position and Follower position."
    )

    parser.add_argument(
        "--name",
        type=str,
        required=False,
        help="The name of the node in the dataflow.",
        default="replay-to-lcr",
    )
    parser.add_argument(
        "--follower-control",
        type=str,
        help="The configuration file for controlling the follower.",
        default=None,
    )

    args = parser.parse_args()

    if not os.environ.get("FOLLOWER_CONTROL") and args.follower_control is None:
        raise ValueError(
            "The follower control is not set. Please set the configuration of the follower in the environment "
            "variables or as an argument."
        )

    with open(
        os.environ.get("FOLLOWER_CONTROL")
        if args.follower_control is None
        else args.follower_control
    ) as file:
        follower_control = json.load(file)
        load_control_table_from_json_conversion_tables(
            follower_control, follower_control
        )

    node = Node(args.name)

    follower_initialized = False

    follower_position = None

    for event in node:
        event_type = event["type"]

        if event_type == "INPUT":
            event_id = event["id"]

            if event_id == "leader_position":
                leader_position = event["value"]

                if not follower_initialized:
                    continue

                physical_goal = logical_to_pwm_with_offset_arrow(
                    follower_position, leader_position, follower_control
                )

                node.send_output("follower_goal", physical_goal, event["metadata"])

            elif event_id == "follower_position":
                follower_position = event["value"]
                follower_initialized = True

        elif event_type == "ERROR":
            print("[replay-to-lcr] error: ", event["error"])
            break


if __name__ == "__main__":
    main()
