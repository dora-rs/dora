# Dora Node for detecting objects in images using YOLOv8

This node is used to detect objects in images using YOLOv8.

# YAML

```yaml
  - id: object_detection
    build: pip install ../../node-hub/ultralytics-yolo
    path: ultralytics-yolo
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
image = {
    "width": np.uint32,
    "height": np.uint32,
    "channels": np.uint8,
    "data": np.array  # flattened image data
}

encoded_image = pa.array([image])

decoded_image = {
    "width": np.uint32(encoded_image[0]["width"].as_py()),
    "height": np.uint32(encoded_image[0]["height"].as_py()),
    "channels": np.uint8(encoded_image[0]["channels"].as_py()),
    "data": encoded_image[0]["data"].values.to_numpy().astype(np.uint8)
}

```

# Outputs

- `bbox`: an arrow array containing the bounding boxes, confidence scores, and class names of the detected objects

```Python

bbox = {
    "bbox": np.array,  # flattened array of bounding boxes
    "conf": np.array,  # flat array of confidence scores
    "names": np.array,  # flat array of class names
}

encoded_bbox = pa.array([bbox])

decoded_bbox = {
    "bbox": encoded_bbox[0]["bbox"].values.to_numpy().reshape(-1, 3),
    "conf": encoded_bbox[0]["conf"].values.to_numpy(),
    "names": encoded_bbox[0]["names"].values.to_pylist(),
}
```

## License

This project is licensed under Apache-2.0. Check out [NOTICE.md](../../NOTICE.md) for more information.
