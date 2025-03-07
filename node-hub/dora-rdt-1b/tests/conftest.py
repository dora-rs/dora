import pytest


def pytest_configure() -> None:
    pytest.rdt = None
    pytest.lang_embeddings = None
    pytest.image_processor = None
    pytest.vision_encoder = None
    pytest.image_embeds = None
    pytest.state_elem_mask = None
    pytest.states = None
    pytest.STATE_INDICES = None
