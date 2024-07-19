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
image: {
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

- `bbox`: an arrow array containing the bounding boxes, confidence scores, and class names of the detected objects

```Python

bbox: {
    "bbox": np.array,  # flattened array of bounding boxes
    "conf": np.array,  # flat array of confidence scores
    "names": np.array,  # flat array of class names
}

encoded_bbox = pa.array([bbox])

decoded_bbox = {
    "bbox": encoded_bbox[0]["bbox"].values.to_numpy().reshape(-1, 3),
    "conf": encoded_bbox[0]["conf"].values.to_numpy(),
    "names": encoded_bbox[0]["names"].values.to_numpy(zero_copy_only=False),
}
```

- `text`: Arrow array containing the text to be plotted

```python
text: str

encoded_text = pa.array([text])

decoded_text = encoded_text[0].as_py()
``` 

## License

This project is licensed under Apache-2.0. Check out [NOTICE.md](../../NOTICE.md) for more information.
