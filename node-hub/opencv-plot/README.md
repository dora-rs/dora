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

      tick:
        source: dora/timer/millis/16 # this node display a window, so it's better to deflect the timer, so when the window is closed, the ticks are not sent anymore in the graph
        queue_size: 1

    outputs:
      - tick

    env:
      PLOT_WIDTH: 640 # optional, default is image input width
      PLOT_HEIGHT: 480 # optional, default is image input height
```

# Inputs
- 
- `tick`: empty Arrow array to trigger the capture

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

- `text`: Arrow array containing the text to be plotted

```python
text = {
    "text": str,
    "font_scale": np.float32,
    "color": (np.uint8, np.uint8, np.uint8),
    "thickness": np.uint32,
    "position": (np.uint32, np.uint32)
}

encoded_text = pa.array([text])

decoded_text = {
    "text": encoded_text[0]["text"].as_py(),
    "font_scale": np.float32(encoded_text[0]["font_scale"].as_py()),
    "color": (np.uint8(encoded_text[0]["color"].as_py()[0]),
              np.uint8(encoded_text[0]["color"].as_py()[1]),
              np.uint8(encoded_text[0]["color"].as_py()[2])),
    "thickness": np.uint32(encoded_text[0]["thickness"].as_py()),
    "position": (np.uint32(encoded_text[0]["position"].as_py()[0]),
                 np.uint32(encoded_text[0]["position"].as_py()[1]))
}
``` 

## License

This project is licensed under Apache-2.0. Check out [NOTICE.md](../../NOTICE.md) for more information.
