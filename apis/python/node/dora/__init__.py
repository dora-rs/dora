"""# dora-rs.

This is the dora python client for interacting with dora dataflow.
You can install it via:
```bash
pip install dora-rs
```.
"""

from enum import Enum

from .dora import (
    Node as Node,
)
from .dora import (
    Ros2Context as Ros2Context,
)
from .dora import (
    Ros2Durability as Ros2Durability,
)
from .dora import (
    Ros2Liveliness as Ros2Liveliness,
)
from .dora import (
    Ros2Node as Ros2Node,
)
from .dora import (
    Ros2NodeOptions as Ros2NodeOptions,
)
from .dora import (
    Ros2Publisher as Ros2Publisher,
)
from .dora import (
    Ros2QosPolicies as Ros2QosPolicies,
)
from .dora import (
    Ros2Subscription as Ros2Subscription,
)
from .dora import (
    Ros2Topic as Ros2Topic,
)
from .dora import (
    __author__ as __author__,
)
from .dora import (
    __version__ as __version__,
)
from .dora import (
    start_runtime as start_runtime,
)


class DoraStatus(Enum):
    """Dora status to indicate if operator `on_input` loop should be stopped.

    Args:
    ----
        Enum (u8): Status signaling to dora operator to
        stop or continue the operator.

    """

    CONTINUE = 0
    STOP = 1
    STOP_ALL = 2
