import pytest


def test_import_main():
    import ugv_sdk_py

    # from opencv_video_capture.main import main
    # skip test as pyorbbecksdk installation is a bit complicated

    # Check that everything is working, and catch dora Runtime Exception as we're not running in a dora dataflow.
    # with pytest.raises(RuntimeError):
    #    main()
