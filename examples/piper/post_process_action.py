from dora import Node

node = Node()

import numpy as np


import time
import pyarrow as pa

for event in node:
    if event["type"] == "INPUT":
        actions = event["value"].to_numpy().copy().reshape((64, 14))

        for action in actions:
            gripper_left = action[6]
            gripper_right = action[13]
            if gripper_right < 0.45:
                action[13] = 0.3
            else:
                action[13] = 0.6

            if gripper_left < 0.45:
                action[6] = 0.3
            else:
                action[6] = 0.6

            node.send_output("jointstate_left", pa.array(action[:7], type=pa.float32()))
            node.send_output(
                "jointstate_right", pa.array(action[7:], type=pa.float32())
            )
            time.sleep(0.005)
        print(actions)
