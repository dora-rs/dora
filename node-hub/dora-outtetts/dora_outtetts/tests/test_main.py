"""TODO: Add docstring."""

import os

import pytest

from dora_outtetts.main import load_interface, main

CI = os.getenv("CI", "false") in ["True", "true"]


def test_import_main():
    """TODO: Add docstring."""
    with pytest.raises(RuntimeError):
        main([])


def test_load_interface():
    """TODO: Add docstring."""
    try:
        interface = load_interface()
    except RuntimeError:
        # Error raised by MPS out of memory.
        if CI:
            interface = "ok"

    assert interface is not None
