# Dora Node for detecting objects in images using YOLOv8

This node is used to detect objects in images using YOLOv8.

# YAML

```yaml
- id: object_detection
  build: pip install ../../node-hub/dora-yolo
  path: dora-yolo
  inputs:
    image: webcam/image

  outputs:
    - bbox
  env:
    MODEL: yolov5n.pt
```

# Inputs

- `image`: Arrow array containing the base image

```python
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

# Outputs

- `bbox`: an arrow array containing the bounding boxes, confidence scores, and class names of the detected objects

```Python

bbox: {
    "bbox": np.array,  # flattened array of bounding boxes
    "conf": np.array,  # flat array of confidence scores
    "labels": np.array,  # flat array of class names
}

encoded_bbox = pa.array([bbox], {"format": "xyxy"})

decoded_bbox = {
    "bbox": encoded_bbox[0]["bbox"].values.to_numpy().reshape(-1, 4),
    "conf": encoded_bbox[0]["conf"].values.to_numpy(),
    "labels": encoded_bbox[0]["labels"].values.to_numpy(zero_copy_only=False),
}
```

## Example

Check example at [examples/python-dataflow](examples/python-dataflow)

## License

This project is licensed under Apache-2.0. Check out [NOTICE.md](../../NOTICE.md) for more information.
