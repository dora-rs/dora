"""
Replay Client: This node is used to represent a leader robot and send a sequence of goals to the dataflow,
reading a dataset of actions and joints from a specific episode.
"""

import os
import argparse

import pyarrow as pa
import pandas as pd

from dora import Node


def joints_values_to_arrow(joints, values):
    return pa.StructArray.from_arrays(
        arrays=[joints, values],
        names=["joints", "values"],
        fields=None,
        mask=None,
        memory_pool=None,
    )


class Client:

    def __init__(self, config: dict[str, any]):
        self.config = config

        self.node = Node(config["name"])

        dataset = pd.read_parquet(config["episode_path"] + "/dataset.parquet")

        # Filter the dataset to only keep rows from the episode
        dataset = dataset[dataset["episode_index"] == config["episode_id"]]

        self.action = dataset["action"]
        self.joints = dataset["joints"]
        self.frame = 0

    def run(self):
        for event in self.node:
            event_type = event["type"]

            if event_type == "INPUT":
                event_id = event["id"]

                if event_id == "pull_position":
                    if self.pull_position(self.node, event["metadata"]):
                        break
                elif event_id == "end":
                    break

            elif event_type == "ERROR":
                raise ValueError("An error occurred in the dataflow: " + event["error"])

        self.node.send_output("end", pa.array([]))

    def pull_position(self, node, metadata) -> bool:
        if self.frame >= len(self.action):
            return True

        action = self.action.iloc[self.frame]
        joints = self.joints.iloc[self.frame]

        position = joints_values_to_arrow(joints, pa.array(action, type=pa.float32()))

        self.frame += 1

        node.send_output("position", position, metadata)


def main():
    # Handle dynamic nodes, ask for the name of the node in the dataflow
    parser = argparse.ArgumentParser(
        description="Replay Client: This node is used to replay a sequence of goals for a followee robot."
    )

    parser.add_argument(
        "--name",
        type=str,
        required=False,
        help="The name of the node in the dataflow.",
        default="replay_client",
    )
    parser.add_argument(
        "--path",
        type=str,
        required=False,
        help="The path to the episode dataset.",
        default=None,
    )
    parser.add_argument(
        "--episode",
        type=int,
        required=False,
        help="The episode id to replay.",
        default=None,
    )

    args = parser.parse_args()

    if (not os.getenv("PATH") and args.path is None) or (
        not os.getenv("EPISODE") and args.episode is None
    ):
        raise ValueError("The environment variables PATH and EPISODE_ID must be set.")

    if not isinstance(int(os.getenv("EPISODE")), int):
        raise ValueError("The environment variable EPISODE_ID must be an integer.")

    # Create configuration
    config = {
        "name": args.name,
        "episode_path": os.getenv("PATH", args.path),
        "episode_id": int(os.getenv("EPISODE", args.episode)),
    }

    print("Replay Client Configuration: ", config, flush=True)

    client = Client(config)
    client.run()


if __name__ == "__main__":
    main()
