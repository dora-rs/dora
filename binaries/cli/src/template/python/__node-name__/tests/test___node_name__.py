"""Test module for __node_name__ package."""

import pyarrow as pa
import pytest

from dora.testing import MockNode


def test_import_main():
    """Test importing and running the main function."""
    from __node_name__.main import main

    # Check that everything is working, and catch Dora RuntimeError
    # as we're not running in a Dora dataflow.
    with pytest.raises(RuntimeError):
        main()


def test_node_logic_with_mock():
    """Test node logic using MockNode (no daemon required)."""
    node = MockNode([("tick", pa.array([0]))])
    for event in node:
        if event["type"] == "INPUT" and event["id"] == "tick":
            # Replace with your node's actual logic
            node.send_output("my_output_id", pa.array([1, 2, 3]))
    assert "my_output_id" in node.outputs
    assert node.outputs["my_output_id"][0].to_pylist() == [1, 2, 3]
