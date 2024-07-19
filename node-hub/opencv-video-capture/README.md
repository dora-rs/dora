# Dora Node for capturing video with OpenCV

This node is used to capture video from a camera using OpenCV.

# YAML

```yaml
- id: opencv-video-capture
  build: pip install ../../node-hub/opencv-video-capture
  path: opencv-video-capture
  inputs:
    tick: dora/timer/millis/16 # try to capture at 60fps
  outputs:
    - image: # the captured image

  env:
    PATH: 0 # optional, default is 0

    IMAGE_WIDTH: 640 # optional, default is video capture width
    IMAGE_HEIGHT: 480 # optional, default is video capture height
```

# Inputs

- `tick`: empty Arrow array to trigger the capture

# Outputs

- `image`: an arrow array containing the captured image

```Python

image: {
    "width": np.uint32,
    "height": np.uint32,
    "encoding": str,
    "data": np.array  # flattened image data
}

encoded_image = pa.array([image])

decoded_image = {
    "width": np.uint32(encoded_image[0]["width"]),
    "height": np.uint32(encoded_image[0]["height"]),
    "encoding": encoded_image[0]["encoding"].as_py(),
    "data": encoded_image[0]["data"].values.to_numpy().astype(np.uint8)
}
```

## License

This project is licensed under Apache-2.0. Check out [NOTICE.md](../../NOTICE.md) for more information.
