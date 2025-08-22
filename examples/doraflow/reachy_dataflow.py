from doraflow import Dataflow

with Dataflow(name="reachy-dataflow") as df:
    # Microphone
    microphone = df.add_node(
        id="dora-microphone",
        build="pip install -e ../../node-hub/dora-microphone",
        path="dora-microphone",
    )
    microphone.add_input("tick", "dora/timer/millis/2000")

    # Voice Activity Detection
    vad = df.add_node(
        id="dora-vad",
        build="pip install -e ../../node-hub/dora-vad",
        path="dora-vad",
    )
    df.add_edge(microphone, "audio", vad, "audio")

    # Speech to Text
    whisper = df.add_node(
        id="dora-distil-whisper",
        build="pip install -e ../../node-hub/dora-distil-whisper",
        path="dora-distil-whisper",
        env={"TARGET_LANGUAGE": "english", "TRANSLATE": True},
    )
    df.add_edge(vad, "audio", whisper, "input")

    # Reachy2 Mobile Base
    reachy_base = df.add_node(
        id="reachy-mobile-base",
        build="pip install -e ../../node-hub/dora-reachy2",
        path="dora-reachy2-mobile-base",
        env={"ROBOT_IP": "172.17.134.85"},
    )

    # Reachy2 Left Arm
    reachy_left_arm = df.add_node(
        id="reachy-left-arm",
        build="pip install -e ../../node-hub/dora-reachy2",
        path="dora-reachy2-left-arm",
        env={"ROBOT_IP": "172.17.134.85"},
    )

    # Reachy2 Right Arm
    reachy_right_arm = df.add_node(
        id="reachy-right-arm",
        build="pip install -e ../../node-hub/dora-reachy2",
        path="dora-reachy2-right-arm",
        env={"ROBOT_IP": "172.17.134.85"},
    )

    # Reachy2 Camera
    reachy_camera = df.add_node(
        id="reachy-camera",
        build="pip install -e ../../node-hub/dora-reachy2",
        path="dora-reachy2-camera",
        env={"ROBOT_IP": "172.17.134.85"},
    )
    reachy_camera.add_input("tick", "dora/timer/millis/50")

    # Reachy2 Head
    reachy_head = df.add_node(
        id="reachy-head",
        build="pip install -e ../../node-hub/dora-reachy2",
        path="dora-reachy2-head",
        env={"ROBOT_IP": "172.17.134.85"},
    )

    # Plot
    plot = df.add_node(
        id="plot",
        build="pip install -e ../../node-hub/dora-rerun",
        path="dora-rerun",
        env={"RERUN_MEMORY_LIMIT": "5%"},
    )

    # Qwen Vision Language Model
    qwenvl = df.add_node(
        id="dora-qwenvl",
        build="pip install -e ../../node-hub/dora-qwen2-5-vl",
        path="dora-qwen2-5-vl",
        env={
            "DEFAULT_QUESTION": "grab human.",
            "IMAGE_RESIZE_RATIO": "0.5",
        },
    )
    qwenvl.add_input("text_1", "dora/timer/millis/500")

    # Parse Bbox
    parse_bbox = df.add_node(
        id="parse_bbox",
        path="parse_bbox.py",
        env={"IMAGE_RESIZE_RATIO": "0.5"},
    )

    # Box Coordinates
    box_coordinates = df.add_node(
        id="box_coordinates",
        build="pip install -e ../../node-hub/dora-object-to-pose",
        path="dora-object-to-pose",
    )

    # Keyboard
    keyboard = df.add_node(
        id="keyboard",
        build="pip install -e ../../node-hub/dora-keyboard",
        path="dora-keyboard",
    )
    keyboard.add_input("tick", "dora/timer/millis/1000")

    # State Machine
    state_machine = df.add_node(
        id="state_machine",
        path="state_machine.py",
        env={
            "ACTIVATION_WORDS": "grab pick give output take catch grabs picks gives output takes catches have"
        },
    )

    # Edges
    df.add_edge(state_machine, "action_base", reachy_base, "action_base")
    df.add_edge(state_machine, "action_l_arm", reachy_left_arm, "pose")
    df.add_edge(state_machine, "action_r_arm", reachy_right_arm, "pose")
    df.add_edge(parse_bbox, "bbox_face", reachy_head, "boxes2d")
    df.add_edge(state_machine, "look", reachy_head, "look")
    df.add_edge(reachy_camera, "image_right", plot, "camera_left/image_right")
    df.add_edge(reachy_camera, "image_depth", plot, "camera_torso/image")
    df.add_edge(qwenvl, "text", plot, "text_response")
    df.add_edge(whisper, "text", plot, "text_whisper")
    df.add_edge(parse_bbox, "bbox", plot, "camera_torso/boxes2d")
    df.add_edge(parse_bbox, "bbox_face", plot, "camera_left/boxes2d_face")
    df.add_edge(reachy_camera, "image_depth", qwenvl, "image_depth")
    df.add_edge(reachy_camera, "image_left", qwenvl, "image_left")
    df.add_edge(state_machine, "text_vlm", qwenvl, "text_2", queue_size=10)
    df.add_edge(qwenvl, "text", parse_bbox, "text")
    df.add_edge(reachy_camera, "depth", box_coordinates, "depth")
    df.add_edge(parse_bbox, "bbox", box_coordinates, "boxes2d")
    df.add_edge(whisper, "text", state_machine, "text")
    df.add_edge(reachy_base, "response_base", state_machine, "response_base")
    df.add_edge(reachy_right_arm, "response_r_arm", state_machine, "response_r_arm")
    df.add_edge(reachy_left_arm, "response_l_arm", state_machine, "response_l_arm")
    df.add_edge(box_coordinates, "pose", state_machine, "pose")

    # Generate the YAML file
    df.to_yaml("reachy_dataflow.yml")

    print("Generated reachy_dataflow.yml")

    # Visualize the dataflow
    df.visualize()
