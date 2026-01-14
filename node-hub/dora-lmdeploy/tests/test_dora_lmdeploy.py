import sys
import unittest
from unittest.mock import MagicMock, patch
import os

# mock deps before import
sys.modules["dora"] = MagicMock()
sys.modules["lmdeploy"] = MagicMock()
sys.modules["pyarrow"] = MagicMock()
sys.modules["numpy"] = MagicMock()
sys.modules["PIL"] = MagicMock()

# adjust path to find module
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import dora_lmdeploy

class TestDoraLmdeploy(unittest.TestCase):
    @patch("dora_lmdeploy.pipeline")
    @patch("dora_lmdeploy.Node")
    def test_text_inference(self, mock_node_cls, mock_pipeline_cls):
        mock_node_instance = mock_node_cls.return_value
        
        # 1. input text event
        event_text = {
            "type": "INPUT",
            "id": "text",
            "value": MagicMock()
        }
        event_text["value"][0].as_py.return_value = "Hello World"
        
        # 2. simulate loop
        mock_node_instance.__iter__.return_value = [event_text]
        
        # setup mock pipeline
        mock_pipe_instance = mock_pipeline_cls.return_value
        mock_response = MagicMock()
        mock_response.text = "Hello there!"
        mock_pipe_instance.side_effect = lambda x: mock_response

        dora_lmdeploy.main()
        
        # verify call and output
        mock_pipe_instance.assert_called_with("Hello World")
        self.assertTrue(mock_node_instance.send_output.called)
        args, _ = mock_node_instance.send_output.call_args
        self.assertEqual(args[0], "response")
        
    @patch("dora_lmdeploy.pipeline")
    @patch("dora_lmdeploy.Node")
    @patch("dora_lmdeploy.Image")
    def test_multimodal_inference(self, mock_image_cls, mock_node_cls, mock_pipeline_cls):
        mock_node_instance = mock_node_cls.return_value
        
        # event 1: image
        event_image = {
            "type": "INPUT",
            "id": "image",
            "value": MagicMock()
        }
        event_image["value"].to_pybytes.return_value = b"fake_image_bytes"
        
        # event 2: text
        event_text = {
            "type": "INPUT",
            "id": "text",
            "value": MagicMock()
        }
        event_text["value"][0].as_py.return_value = "Describe this."
        
        mock_node_instance.__iter__.return_value = [event_image, event_text]
        
        # setup mock pipeline and image
        mock_pipe_instance = mock_pipeline_cls.return_value
        mock_response = MagicMock()
        mock_response.text = "A beautiful sunset."
        mock_pipe_instance.side_effect = lambda x: mock_response

        mock_pil_image = MagicMock()
        mock_image_cls.open.return_value = mock_pil_image

        dora_lmdeploy.main()
        
        # verify logic
        mock_image_cls.open.assert_called()
        mock_pipe_instance.assert_called_with(("Describe this.", mock_pil_image))
        mock_node_instance.send_output.assert_called()

if __name__ == "__main__":
    unittest.main()
