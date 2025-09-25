import os

from dora import build, run
from dora.builder import DataflowBuilder

# Get the directory of the current script
script_dir = os.path.dirname(os.path.abspath(__file__))

# Initialize a new dataflow
dataflow = DataflowBuilder(name="yolo-dataflow")
# Define the nodes
camera = dataflow.add_node(
    id="camera",
    path="opencv-video-capture",
    build="pip install opencv-video-capture",
    env={
        "CAPTURE_PATH": "0",
        "IMAGE_WIDTH": "640",
        "IMAGE_HEIGHT": "480",
    },
)
camera.add_input("tick", "dora/timer/millis/20")
camera_image = camera.add_output("image")

object_detection = dataflow.add_node(
    id="object-detection",
    path="dora-yolo",
    build="pip install dora-yolo",
)
object_detection.add_input("image", camera_image)
bbox_output = object_detection.add_output("bbox")

plot = dataflow.add_node(
    id="plot",
    path="opencv-plot",
    build="pip install opencv-plot",
)
# Connect the nodes using outputs
plot.add_input("image", camera_image)
plot.add_input("boxes2d", bbox_output)

# Generate the YAML file
dataflow.to_yaml("dataflow.yml")
print("Generated dataflow.yml")

# If env var NO_BUILD is set, skip the build and run steps
if not os.getenv("NO_BUILD"):
    build("dataflow.yml", uv=True)
else:
    print("Skipping build and run steps due to NO_BUILD env var")
print("Running dataflow.yml")
run("dataflow.yml", uv=True)
