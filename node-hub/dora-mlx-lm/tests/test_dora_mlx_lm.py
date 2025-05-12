import pytest


def test_mlx_lm_node():
    """
    Test the import and execution of the mlx_lm_node function.

    This test verifies that the mlx_lm_node function can be imported from the dora_mlx_lm module
    and checks that calling it outside a DORA dataflow raises a RuntimeError, as expected.
    """
    from dora_mlx_lm.main import main

    # Check that calling the node function raises a RuntimeError, as it requires a DORA dataflow environment.
    with pytest.raises(RuntimeError):
        main()