from doraflow import Dataflow

# Initialize a new dataflow
with Dataflow(name="yolo-dataflow") as df:
    # Define the nodes
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

    # Connect the nodes
    df.add_edge(camera, "image", object_detection, "image")
    df.add_edge(camera, "image", plot, "image")
    df.add_edge(object_detection, "bbox", plot, "boxes2d")

    # Generate the YAML file
    df.to_yaml("dataflow.yml")

    print("Generated dataflow.yml")

    # Visualize the dataflow
    df.visualize()
