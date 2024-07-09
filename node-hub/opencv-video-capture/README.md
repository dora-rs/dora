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
      CAMERA_ID: 0 # optional, default is 0
      CAMERA_WIDTH: 640 # optional, default is 640
      CAMERA_HEIGHT: 480 # optional, default is 480
```

