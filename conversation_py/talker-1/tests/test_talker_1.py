"""Test module for talker_1 package."""

import pytest


def test_import_main():
    """Test importing and running the main function."""
    from talker_1.main import main

    # Check that everything is working, and catch Dora RuntimeError
    # as we're not running in a Dora dataflow.
    with pytest.raises(RuntimeError):
        main()
