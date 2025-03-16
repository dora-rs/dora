from dora import Node
import pandas as pd
import pyarrow as pa
import time


TOPIC = "puppet_goal_position"


if __name__ == "__main__":
    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            for action in event["value"]:
                print(action, flush=True)
                action = action.as_py()

                df = pd.read_parquet(action + ".parquet")

                initial_time = df["timestamp_utc"].iloc[0]
                current_time = initial_time
                for index, row in df.iterrows():
                    delta_time = (row["timestamp_utc"] - current_time).microseconds
                    current_time = row["timestamp_utc"]
                    time.sleep(delta_time / 1_000_000)
                    node.send_output(TOPIC, pa.array(row[TOPIC], type=pa.uint32()))
