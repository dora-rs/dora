"""Backward-compatible shim: ``from dora import Node`` works for dora-hub nodes.

This package re-exports the dora Python API under the ``dora`` namespace so
that existing dora-hub nodes and operators work without modification.
"""

from dora import (  # noqa: F401
    DoraStatus,
    Node,
    build,
    run,
)

# ROS2 types are optional — dora may be built without ROS2 support.
# Wrap in try/except so `from dora import Node` works for everyone.
try:
    from dora import (  # noqa: F401
        Ros2Context,
        Ros2Durability,
        Ros2Liveliness,
        Ros2Node,
        Ros2NodeOptions,
        Ros2Publisher,
        Ros2QosPolicies,
        Ros2Subscription,
        Ros2Topic,
    )
except ImportError:
    pass

# start_runtime is provided by the dora-cli package, not the node API.
try:
    from dora import start_runtime  # noqa: F401
except ImportError:
    pass

# Backward-compatible alias used by dora-hub operators.
DoraStatus = DoraStatus
