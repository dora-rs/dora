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
## Image data
image_data: UInt8Array # Example: pa.array(img.ravel())
metadata = {
  "width": 640,
  "height": 480,
  "encoding": str, # bgr8, rgb8
}

## Example
node.send_output(
  image_data, {"width": 640, "height": 480, "encoding": "bgr8"}
  )

## Decoding
storage = event["value"]

metadata = event["metadata"]
encoding = metadata["encoding"]
width = metadata["width"]
height = metadata["height"]

if encoding == "bgr8":
    channels = 3
    storage_type = np.uint8

frame = (
    storage.to_numpy()
    .astype(storage_type)
    .reshape((height, width, channels))
)
```

## Examples

Check example at [examples/python-dataflow](examples/python-dataflow)

## License

This project is licensed under Apache-2.0. Check out [NOTICE.md](../../NOTICE.md) for more information.
