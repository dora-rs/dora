"""TODO: Add docstring."""

import pytest


def test_import_main():
    """TODO: Add docstring."""
    from dora_kokoro_tts.main import main

    # Check that everything is working, and catch dora Runtime Exception as we're not running in a dora dataflow.
    with pytest.raises(RuntimeError):
        main()
