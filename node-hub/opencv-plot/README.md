# Dora Node for plotting data with OpenCV

This node is used to plot a text and a list of bbox on a base image (ideal for object detection).

# YAML

```yaml
- id: opencv-plot
  build: pip install ../../node-hub/opencv-plot
  path: opencv-plot
  inputs:
    # image: Arrow array of size 1 containing the base image
    # bbox: Arrow array of bbox
    # text: Arrow array of size 1 containing the text to be plotted

  env:
    PLOT_WIDTH: 640 # optional, default is image input width
    PLOT_HEIGHT: 480 # optional, default is image input height
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

- `text`: Arrow array containing the text to be plotted

```python
text: str

encoded_text = pa.array([text])

decoded_text = encoded_text[0].as_py()
```

## Example

Check example at [examples/python-dataflow](examples/python-dataflow)

## License

This project is licensed under Apache-2.0. Check out [NOTICE.md](../../NOTICE.md) for more information.
