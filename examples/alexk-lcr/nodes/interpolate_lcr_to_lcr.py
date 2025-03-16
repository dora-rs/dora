import os
import argparse
import json

import pyarrow as pa
import pyarrow.compute as pc

from dora import Node

from pwm_position_control.transform import (
    wrap_joints_and_values,
    pwm_to_logical_arrow,
    logical_to_pwm_with_offset_arrow,
)
from pwm_position_control.load import load_control_table_from_json_conversion_tables


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
        default="lcr-to-lcr",
    )
    parser.add_argument(
        "--leader-control",
        type=str,
        help="The configuration file for controlling the leader.",
        default=None,
    )
    parser.add_argument(
        "--follower-control",
        type=str,
        help="The configuration file for controlling the follower.",
        default=None,
    )

    args = parser.parse_args()

    if not os.environ.get("LEADER_CONTROL") and args.leader_control is None:
        raise ValueError(
            "The leader control is not set. Please set the configuration of the leader in the environment variables or "
            "as an argument."
        )

    if not os.environ.get("FOLLOWER_CONTROL") and args.follower_control is None:
        raise ValueError(
            "The follower control is not set. Please set the configuration of the follower in the environment "
            "variables or as an argument."
        )

    with open(
        os.environ.get("LEADER_CONTROL")
        if args.leader_control is None
        else args.leader_control
    ) as file:
        leader_control = json.load(file)
        load_control_table_from_json_conversion_tables(leader_control, leader_control)

    with open(
        os.environ.get("FOLLOWER_CONTROL")
        if args.follower_control is None
        else args.follower_control
    ) as file:
        follower_control = json.load(file)
        load_control_table_from_json_conversion_tables(
            follower_control, follower_control
        )

    initial_mask = [
        True if leader_control[joint]["goal_position"] is not None else False
        for joint in leader_control.keys()
    ]
    logical_leader_initial_goal = wrap_joints_and_values(
        [
            joint
            for joint in leader_control.keys()
            if leader_control[joint]["goal_position"] is not None
        ],
        [
            leader_control[joint]["goal_position"]
            for joint in leader_control.keys()
            if leader_control[joint]["goal_position"] is not None
        ],
    )

    node = Node(args.name)

    leader_initialized = False
    follower_initialized = False

    follower_position = None

    for event in node:
        event_type = event["type"]

        if event_type == "INPUT":
            event_id = event["id"]

            if event_id == "leader_position":
                leader_position = event["value"]

                if not leader_initialized:
                    leader_initialized = True

                    pwm_goal = logical_to_pwm_with_offset_arrow(
                        leader_position.filter(initial_mask),
                        logical_leader_initial_goal,
                        leader_control,
                    )

                    node.send_output("leader_goal", pwm_goal, event["metadata"])

                if not follower_initialized:
                    continue

                leader_position = pwm_to_logical_arrow(leader_position, leader_control)

                interpolation = pa.array([1, 1, 1, 1, 1, 700 / 450], type=pa.float32())

                logical_goal = wrap_joints_and_values(
                    leader_position.field("joints"),
                    pc.multiply(leader_position.field("values"), interpolation),
                )

                pwm_goal = logical_to_pwm_with_offset_arrow(
                    follower_position, logical_goal, follower_control
                )

                node.send_output("follower_goal", pwm_goal, event["metadata"])

            elif event_id == "follower_position":
                follower_position = event["value"]
                follower_initialized = True

        elif event_type == "ERROR":
            print("[lcr-to-lcr] error: ", event["error"])
            break


if __name__ == "__main__":
    main()
