import time

from dora_watermark import load


def run(inputs):
    if "control_status" not in inputs.keys():
        return {}
    _, timestamps = load(inputs, "control_status")

    previous_timestamp = time.time()
    for timestamp in timestamps:
        if timestamp[0] == "carla_source_operator":
            print(f"Global time cost: {timestamp[1] - previous_timestamp:.2f}")
            previous_timestamp = timestamp[1]
        else:
            print(
                f"{timestamp[0]} latency: {timestamp[1] - previous_timestamp:.2f}"
            )
            previous_timestamp = timestamp[1]
    return {}
