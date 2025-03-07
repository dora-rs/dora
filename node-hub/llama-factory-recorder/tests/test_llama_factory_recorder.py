import pytest


def test_import_main() -> None:
    from llama_factory_recorder.main import main

    # Check that everything is working, and catch dora Runtime Exception as we're not running in a dora dataflow.
    with pytest.raises(RuntimeError):
        main()
