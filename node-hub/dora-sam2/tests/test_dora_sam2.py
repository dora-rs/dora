"""TODO: Add docstring."""

import os

import pytest

CI = os.getenv("CI", "false") in ["True", "true"]


def test_import_main():
    """TODO: Add docstring."""
    if CI:
        # Skip test as test requires Nvidia GPU
        return

    from dora_sam2.main import main

    # Check that everything is working, and catch the dora Runtime Exception 
    # since we're not running in a dora dataflow.

    with pytest.raises(RuntimeError):
        main()
