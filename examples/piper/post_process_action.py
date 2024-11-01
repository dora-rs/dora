from dora import Node

node = Node()

import numpy as np


import time
import pyarrow as pa

for event in node:
    if event["type"] == "INPUT":
        actions = event["value"].to_numpy().reshape((64, 14))

        # Skip action to only keep 8 spread action
        actions = actions[[0, 8, 16, 24, 32, 40, 48, 56], :]

        for action in actions:
            node.send_output("jointstate_left", pa.array(action[:7], type=pa.float32()))
            node.send_output(
                "jointstate_right", pa.array(action[7:], type=pa.float32())
            )
            time.sleep(0.005)
        print(actions)
