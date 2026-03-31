"""Verify that dora-hub import patterns work via the dora compatibility shim.

These tests ensure that existing dora-hub nodes can be used with adora
without any import changes.
"""


def test_from_dora_import_node():
    """Dora-hub pattern: `from dora import Node`."""
    from dora import Node

    assert callable(Node)


def test_from_dora_import_dora_status():
    """Dora-hub pattern: `from dora import DoraStatus`."""
    from dora import DoraStatus

    assert hasattr(DoraStatus, "CONTINUE")
    assert hasattr(DoraStatus, "STOP")
    assert hasattr(DoraStatus, "STOP_ALL")
    assert DoraStatus.CONTINUE.value == 0
    assert DoraStatus.STOP.value == 1
    assert DoraStatus.STOP_ALL.value == 2


def test_dora_status_is_adora_status():
    """DoraStatus must be the same class as AdoraStatus."""
    from adora import AdoraStatus
    from dora import DoraStatus

    assert DoraStatus is AdoraStatus


def test_from_dora_import_node_and_status():
    """Dora-hub combined pattern: `from dora import DoraStatus, Node`."""
    from dora import DoraStatus, Node

    assert callable(Node)
    assert DoraStatus.CONTINUE.value == 0


def test_dora_node_same_as_adora_node():
    """The Node class from both packages must be identical."""
    from adora import Node as AdoraNode
    from dora import Node as DoraNode

    assert DoraNode is AdoraNode


def test_dora_ros2_types_accessible():
    """ROS2 types should be accessible from the dora shim when available."""
    try:
        from dora import (
            Ros2Context,
            Ros2NodeOptions,
        )

        assert Ros2Context is not None
        assert Ros2NodeOptions is not None
    except ImportError:
        # ROS2 support is optional — not an error if unavailable.
        pass


def test_dora_build_and_run_importable():
    """Top-level functions used by dora-hub scripts."""
    from dora import build, run

    assert callable(build)
    assert callable(run)
