"""Test module for dora_pytorch_kinematics package."""

import pytest


def test_import_main():
    """Test importing and running the main function."""
    from dora_pytorch_kinematics.main import main

    # Check that everything is working, and catch Dora RuntimeError
    # as we're not running in a Dora dataflow.
    with pytest.raises(RuntimeError):
        main()
