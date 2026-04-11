"""
# dora-rs.

This is the dora python client for interacting with dora dataflow.
You can install it via:
```bash
pip install dora-rs
```.
"""

from enum import Enum

from .dora import *
from .dora import (
    Node,
    Ros2Context,
    Ros2Durability,
    Ros2Liveliness,
    Ros2Node,
    Ros2NodeOptions,
    Ros2Publisher,
    Ros2QosPolicies,
    Ros2Subscription,
    Ros2Topic,
    __author__,
    __version__,
)

# start_runtime is provided by the dora-cli package, not the node API.
# Import it conditionally so that `from dora import Node` works without
# requiring the full CLI to be installed.
try:
    from .dora import start_runtime
except ImportError:
    pass


class DoraStatus(Enum):
    """Dora status to indicate if operator `on_input` loop should be stopped.

    Args:
        Enum (u8): Status signaling to dora operator to
        stop or continue the operator.

    """

    CONTINUE = 0
    STOP = 1
    STOP_ALL = 2
