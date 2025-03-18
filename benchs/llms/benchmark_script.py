"""TODO: Add docstring."""

import argparse
import ast

# Create an empty csv file with header in the current directory if file does not exist
import csv
import os
import time

import numpy as np
import pyarrow as pa
from dora import Node


def write_to_csv(filename, header, row):
    """
    Create a CSV file with a header if it does not exist, and write a row to it.
    If the file exists, append the row to the file.

    :param filename: Name of the CSV file.
    :param header: List of column names to use as the header.
    :param row: List of data to write as a row in the CSV file.
    """
    file_exists = os.path.exists(filename)

    with open(
        filename, mode="a" if file_exists else "w", newline="", encoding="utf8"
    ) as file:
        writer = csv.writer(file)

        # Write the header if the file is being created
        if not file_exists:
            writer.writerow(header)
            print(f"File '{filename}' created with header: {header}")

        # Write the row
        writer.writerow(row)
        print(f"Row written to '{filename}': {row}")


def main():
    # Handle dynamic nodes, ask for the name of the node in the dataflow, and the same values as the ENV variables.
    """TODO: Add docstring."""
    parser = argparse.ArgumentParser(description="Simple arrow sender")

    parser.add_argument(
        "--name",
        type=str,
        required=False,
        help="The name of the node in the dataflow.",
        default="pyarrow-sender",
    )
    parser.add_argument(
        "--data",
        type=str,
        required=False,
        help="Arrow Data as string.",
        default=None,
    )

    args = parser.parse_args()

    data = os.getenv("DATA", args.data)

    node = Node(
        args.name,
    )  # provide the name to connect to the dataflow if dynamic node
    name = node.dataflow_descriptor()["nodes"][1]["path"]

    if data is None:
        raise ValueError(
            "No data provided. Please specify `DATA` environment argument or as `--data` argument",
        )
    try:
        data = ast.literal_eval(data)
    except Exception:  # noqa
        print("Passing input as string")

    if isinstance(data, (str, int, float)):
        data = pa.array([data])
    else:
        data = pa.array(data)  # initialize pyarrow array

    durations = []
    speed = []
    for _ in range(10):
        start_time = time.time()
        node.send_output("data", data)
        event = node.next()
        duration = time.time() - start_time
        if event is not None and event["type"] == "INPUT":
            text = event["value"][0].as_py()
            tokens = event["metadata"].get("tokens", 6)
            assert "this is a test" in text.lower(), (
                f"Expected 'This is a test', got {text}"
            )
            durations.append(duration)
            speed.append(tokens / duration)
            time.sleep(0.1)
    durations = np.array(durations)
    speed = np.array(speed)
    print(
        f"\nAverage duration: {sum(durations) / len(durations)}"
        + f"\nMax duration: {max(durations)}"
        + f"\nMin duration: {min(durations)}"
        + f"\nMedian duration: {np.median(durations)}"
        + f"\nMedian frequency: {1 / np.median(durations)}"
        + f"\nAverage speed: {sum(speed) / len(speed)}"
        + f"\nMax speed: {max(speed)}"
        + f"\nMin speed: {min(speed)}"
        + f"\nMedian speed: {np.median(speed)}"
        + f"\nTotal tokens: {tokens}"
    )
    write_to_csv(
        "benchmark.csv",
        [
            "path",
            "date",
            "average_duration(s)",
            "max_duration(s)",
            "min_duration(s)",
            "median_duration(s)",
            "median_frequency(Hz)",
            "average_speed(tok/s)",
            "max_speed(tok/s)",
            "min_speed(tok/s)",
            "median_speed(tok/s)",
            "total_tokens",
        ],
        [
            name,
            time.strftime("%Y-%m-%d %H:%M:%S"),
            sum(durations) / len(durations),
            max(durations),
            min(durations),
            np.median(durations),
            1 / np.median(durations),
            sum(speed) / len(speed),
            max(speed),
            min(speed),
            np.median(speed),
            tokens,
        ],
    )


if __name__ == "__main__":
    main()
