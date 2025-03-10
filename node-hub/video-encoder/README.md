## Camera Node for OpenCV compatible cameras

Simple Camera node that uses OpenCV to capture images from a camera. The node can be configured to use a specific camera
id, width and height.
It then sends the images to the dataflow.

## YAML Configuration

````YAML
nodes:
  - id: video_encoder
    path: encoder.py # modify this to the relative path from the graph file to the client script
    inputs:
    # image: some image from other node
    # episode_index: some episode index from other node
    outputs:
      - image

    env:
      VIDEO_NAME: cam_up
      VIDEO_WIDTH: 640
      VIDEO_HEIGHT: 480
      FPS: 30
````

## License

This library is licensed under the [Apache License 2.0](../../LICENSE).