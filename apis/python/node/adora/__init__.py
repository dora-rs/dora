""" 
# adora-rs.

This is the adora python client for interacting with adora dataflow.
You can install it via:
```bash
pip install adora-rs
```.
"""

from enum import Enum

from .adora import *
from .adora import (
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

# start_runtime is provided by the adora-cli package, not the node API.
# Import it conditionally so that `from adora import Node` works without
# requiring the full CLI to be installed.
try:
    from .adora import start_runtime
except ImportError:
    pass


class AdoraStatus(Enum):
    """Adora status to indicate if operator `on_input` loop should be stopped.

    Args:
        Enum (u8): Status signaling to adora operator to
        stop or continue the operator.

    """

    CONTINUE = 0
    STOP = 1
    STOP_ALL = 2
