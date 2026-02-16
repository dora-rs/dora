"""Test module for __node_name__ package."""

import pytest


def test_import_main():
    """Test importing and running the main function."""
    from __node_name__.main import main

    # Check that everything is working, and catch Adora RuntimeError
    # as we're not running in a Adora dataflow.
    with pytest.raises(RuntimeError):
        main()
