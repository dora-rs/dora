import pytest
import dora


def test_context_defaults_to_dds():
    context = dora.Ros2Context(ros_paths=[])
    assert context.transport_kind == "dds"


def test_existing_positional_context_construction_remains_valid():
    assert dora.Ros2Context([]).transport_kind == "dds"


def test_zenoh_transport_requires_known_profile():
    with pytest.raises(ValueError, match="compatibility"):
        dora.Ros2Transport.zenoh("automatic")


def test_context_transport_is_read_only():
    context = dora.Ros2Context(ros_paths=[])
    with pytest.raises(AttributeError):
        context.transport_kind = "zenoh"
