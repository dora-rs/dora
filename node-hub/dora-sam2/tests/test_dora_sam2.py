import os

import pytest

CI = os.getenv("CI", "false") in ["True", "true"]


def test_import_main():
    if CI:
        # Skip test as test requires Nvidia GPU
        return

    from dora_sam2.main import main

    # Check that everything is working, and catch dora Runtime Exception as we're not running in a dora dataflow.
    with pytest.raises(RuntimeError):
        main()
