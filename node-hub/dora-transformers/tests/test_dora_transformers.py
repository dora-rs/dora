"""Unit test for importing and executing the `main` function from `dora_transformers.main`.

This test ensures that the `main` function is correctly imported and raises a `RuntimeError`
when executed outside of a Dora runtime environment.
"""

import pytest


def test_import_main():
    """Test the import of the `main` function from `dora_transformers.main`.

    This test ensures that the `main` function is correctly imported and executed.
    Since it relies on a Dora dataflow, it is expected to raise a `RuntimeError`
    when run in an isolated test environment.

    Expected Behavior:
    - The test should pass if `RuntimeError` is raised, indicating that
      the function is properly imported but fails due to missing Dora runtime.

    Raises
    ------
    - RuntimeError: Expected since Dora runtime is not running.

    """
    from dora_transformers.main import main

    # Check that everything is working, and catch Dora Runtime Exception
    # as we're not running in a Dora dataflow.
    with pytest.raises(RuntimeError):
        main()
