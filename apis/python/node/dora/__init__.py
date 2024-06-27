""" 
# dora-rs
This is the dora python client for interacting with dora dataflow.
You can install it via:
```bash
pip install dora-rs
```
"""

from enum import Enum

from .dora import *

from .dora import (
    Node,
    Ros2Context,
    Ros2Node,
    Ros2NodeOptions,
    Ros2Topic,
    Ros2Publisher,
    Ros2Subscription,
    start_runtime,
    __version__,
    __author__,
    Ros2QosPolicies,
    Ros2Durability,
    Ros2Liveliness,
)


class DoraStatus(Enum):
    """Dora status to indicate if operator `on_input` loop
     should be stopped.
    Args:
        Enum (u8): Status signaling to dora operator to
        stop or continue the operator.
    """

    CONTINUE = 0
    STOP = 1
    STOP_ALL = 2
