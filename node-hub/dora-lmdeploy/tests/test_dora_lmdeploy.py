"""Module for testing Dora LMDeploy functionality.

This module contains tests for verifying the functionality of the Dora LMDeploy component.
"""


import pytest

def test_import_main():
    """Test that the main function in Dora LMDeploy can be imported and executed.

    This test verifies that the `main` function can be imported and runs without errors,
    catching any expected RuntimeError when executed outside a valid Dora dataflow context.
    """
    from dora_lmdeploy.main import main

    with pytest.raises(RuntimeError, match=".*Node.*"):
        main()

