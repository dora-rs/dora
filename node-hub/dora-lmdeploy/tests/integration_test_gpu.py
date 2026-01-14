import os
import time
import pytest
import pyarrow as pa
from dora import Node

# this test requires a gpu and lmdeploy installed
# run with: pytest tests/integration_test_gpu.py
@pytest.mark.skipif(os.environ.get("e2e_gpu_test") != "true", reason="requires gpu")
def test_full_pipeline_real_model():
    # start the node in a separate process or thread usually
    # here we simulate the node behavior by importing main 
    # but strictly speaking, dora runs nodes as processes.
    
    # for a true integration test, we often construct a dataflow.yml
    # and run `dora up` and `dora start`.
    
    # helper to check if model loads
    try:
        from lmdeploy import pipeline
        # use a tiny model for fast testing if available, else standard
        model = "Qwen/Qwen2.5-VL-7B-Instruct"
        pipe = pipeline(model)
        print("Model loaded.")
        
        # run simple inference
        resp = pipe("hello")
        assert resp is not None
        print("Text inference success:", resp)

        # verify multimodal inference
        from PIL import Image
        import numpy as np
        # create valid dummy image
        img = Image.fromarray(np.zeros((64, 64, 3), dtype=np.uint8))
        print("Running multimodal check...")
        resp_img = pipe(("Describe this black image.", img))
        assert resp_img is not None
        print("Multimodal inference success:", resp_img)
        
    except ImportError:
        pytest.fail("lmdeploy not installed")
    except Exception as e:
        pytest.fail(f"Model failed to load: {e}")
