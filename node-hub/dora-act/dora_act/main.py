import argparse
import os
from dora import Node
import pyarrow as pa

from infer_real import *
from infer_sim import *


if __name__ == "__main__":
    scenario = os.getenv("SCENARIO", "real")
    if scenario == "real":
        act_inference_real = ActInferenceReal()
        act_inference_real.inference()
    elif scenario == "sim":
        act_inference_sim = ActInferenceSim()
        act_inference_sim.inference()
    else:
        print(f"do not have sceanrio {scenario}")
        raise NotImplementedError
