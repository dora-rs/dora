from doraflow import Dataflow
import os

# Get the directory of the current script
script_dir = os.path.dirname(os.path.abspath(__file__))

# Initialize a new dataflow
with Dataflow(name="yolo-dataflow") as df:
    # Define the nodes
    camera = df.add_node(
        id="camera",
        path=os.path.abspath(os.path.join(script_dir, "../../node-hub/opencv-video-capture/opencv_video_capture/main.py")),
        build="pip install opencv-python",
        env={
            "CAPTURE_PATH": "0",
            "IMAGE_WIDTH": "640",
            "IMAGE_HEIGHT": "480",
        },
    )
    camera.add_input("tick", "dora/timer/millis/20")

    object_detection = df.add_node(
        id="object-detection",
        path=os.path.abspath(os.path.join(script_dir, "../../node-hub/dora-yolo/dora_yolo/main.py")),
        build="pip install ultralytics",
    )

    plot = df.add_node(
        id="plot",
        path=os.path.abspath(os.path.join(script_dir, "../../node-hub/opencv-plot/opencv_plot/main.py")),
        build="pip install opencv-python numpy pillow pillow-avif-plugin",
    )

    # Connect the nodes
    df.add_edge(camera, "image", object_detection, "image")
    df.add_edge(camera, "image", plot, "image")
    df.add_edge(object_detection, "bbox", plot, "boxes2d")

    # Generate the YAML file
    df.to_yaml("dataflow.yml")

    print("Generated dataflow.yml")

    # Visualize the dataflow
    df.visualize()

    # Run the dataflow
    # df.run()

    # Start the dataflow in the background
    df.start()
    print("Dataflow started in the background. It will run for 20 seconds.")

    import time
    time.sleep(20)

    print("Stopping dataflow...")
    df.stop()
    print("Dataflow stopped.")
