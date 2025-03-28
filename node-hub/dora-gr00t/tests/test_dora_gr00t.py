"""TODO: Add docstring."""

import os
import sys

import pytest

# Add the parent directory to the path to make the package discoverable
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))


def test_import_main():
    """TODO: Add docstring."""
    from dora_gr00t.main import main

    # Check that everything is working, and catch dora Runtime Exception as we're not running in a dora dataflow.
    with pytest.raises(RuntimeError):
        main()
