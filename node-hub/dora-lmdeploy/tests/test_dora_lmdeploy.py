import pytest

def test_import_main():
    """Test that the main function in dora-lmdeploy can be imported and executed."""
    from dora_lmdeploy.main import main

    with pytest.raises(RuntimeError, match=".*Node.*"):
        main()

