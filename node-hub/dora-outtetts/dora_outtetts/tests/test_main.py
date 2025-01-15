import pytest
from dora_outtetts.main import load_interface, main


def test_import_main():
    with pytest.raises(RuntimeError):
        main([])


def test_load_interface():
    interface = load_interface()
    assert interface is not None
