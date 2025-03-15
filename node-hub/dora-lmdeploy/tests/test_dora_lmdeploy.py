"""Tests for the dora-lmdeploy node."""

import pytest


def test_import_main():
    """Test that the main function in dora-lmdeploy can be imported and executed."""
    from dora_lmdeploy.main import main

    # Check that everything is working, and catch Dora RuntimeException as we're not running in a Dora dataflow.
    with pytest.raises(RuntimeError):
        main()
