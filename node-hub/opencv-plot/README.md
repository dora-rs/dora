# Dora Node for plotting data with OpenCV

This node is used to plot a text and a list of bbox on a base image (ideal for object detection).

# YAML

```yaml
  - id: opencv-plot
    build: pip install ../../node-hub/opencv-plot
    path: opencv-plot
    inputs:
      # image: some image from another node, it's the base image 
      # bbox: Arrow array of bbox
      # text: Arrow array of size 1 containing the text to be plotted

      tick:
        source: dora/timer/millis/16 # this node display a window, so it's better to deflect the timer, so when the window is closed, the ticks are not sent anymore in the graph
        queue_size: 1

    outputs:
      - tick

    env:
      IMAGE_WIDTH: 640 # optional, default is 640
      IMAGE_HEIGHT: 480 # optional, default is 480
```

