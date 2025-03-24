"""Test for Dora MLX """

import pytest


def test_import_main():
    """Test that the MLX Dora chatbot script can be imported and run."""
    from dora_mlx.main import main  # Adjust the import path based on your directory structure

    # Expect RuntimeError since we're not running inside a Dora dataflow
    with pytest.raises(RuntimeError):
        main()


