import argparse
import os
from dora import Node
import pyarrow as pa

from infer_sim import *


if __name__ == "__main__":
    scenario = os.getenv("SCENARIO", "real")
    if scenario == "sim":
        act_inference_sim = ActInferenceSim()
        act_inference_sim.inference()
    else:
        print(f"do not have sceanrio {scenario}")
        raise NotImplementedError
