import unittest
import yaml
import os

from doraflow.dataflow import Dataflow

class TestIntegration(unittest.TestCase):

    def test_yolo_dataflow_generation(self):
        # Define the dataflow using the Python API
        with Dataflow(name="yolo-dataflow") as df:
            camera = df.add_node(
                id="camera",
                path="../../node-hub/opencv-video-capture/opencv_video_capture/main.py",
                build="pip install opencv-python",
                env={
                    "CAPTURE_PATH": 0,
                    "IMAGE_WIDTH": 640,
                    "IMAGE_HEIGHT": 480,
                },
            )
            camera.add_input("tick", "dora/timer/millis/20")

            object_detection = df.add_node(
                id="object-detection",
                path="../../node-hub/dora-yolo/dora_yolo/main.py",
                build="pip install ultralytics",
            )

            plot = df.add_node(
                id="plot",
                path="../../node-hub/dora-rerun/dora_rerun/main.py",
                build="pip install rerun",
            )

            df.add_edge(camera, "image", object_detection, "image")
            df.add_edge(camera, "image", plot, "image")
            df.add_edge(object_detection, "bbox", plot, "boxes2d")

        # Generate the YAML string
        generated_yaml_str = df.to_yaml()
        generated_data = yaml.safe_load(generated_yaml_str)

        # Load the expected YAML file
        current_dir = os.path.dirname(os.path.abspath(__file__))
        expected_yaml_path = os.path.join(current_dir, "expected_yolo_dataflow.yml")
        with open(expected_yaml_path, "r") as f:
            expected_data = yaml.safe_load(f)

        # Compare the generated YAML with the expected YAML
        self.assertEqual(generated_data, expected_data)

if __name__ == '__main__':
    unittest.main()
