"""TODO: Add docstring."""

import pytest


def test_import_main():
    """TODO: Add docstring."""
    from dora_pyrealsense.main import main

    # Check that everything is working, and catch the dora Runtime Exception 
# since we're not running in a dora dataflow.
    with pytest.raises(ConnectionError):
        main()
