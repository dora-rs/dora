"""Tests for the dora-phi4 package.

This module contains tests to verify the functionality of the dora-phi4 package,
including model loading, input processing, and error handling.
"""

import pytest


def test_import_main():
    """Test that the main function can be imported and raises appropriate errors.
    
    This test verifies that:
    1. The main function can be imported from the package
    2. Running the main function outside a Dora dataflow raises a RuntimeError
    """
    from dora_phi4.main import main

    # Check that everything is working, and catch dora Runtime Exception as we're not running in a dora dataflow.
    with pytest.raises(RuntimeError):
        main()
