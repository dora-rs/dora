# WebRTC Sink Example

This example demonstrates how to use `dora-gst-webrtc-sink` with `dora-gst-test-source` to stream video via WebRTC.

## Prerequisites

1. Install dora-rs
2. Install GStreamer with WebRTC support:
   ```bash
   # Ubuntu/Debian
   sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
     libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-bad \
     gstreamer1.0-nice gstreamer1.0-tools
   ```

## Running the Demo

1. Run the demo script:
   ```bash
   ./run_demo.sh
   ```

2. Open `webrtc-viewer.html` in a web browser

3. Click the "Connect" button to start viewing the stream

## Components

- **dataflow.yml**: Dora dataflow configuration that connects test video source to WebRTC sink
- **webrtc-viewer.html**: Simple web-based WebRTC client for viewing the stream
- **run_demo.sh**: Script to build and run the demo

## Architecture

```
[dora-gst-test-source] ---(RGB images)---> [dora-gst-webrtc-sink]
                                                    |
                                                    v
                                            WebSocket Server (:8080)
                                                    |
                                                    v
                                            Web Browser Client
```

## Troubleshooting

- If the build fails with "gstreamer-webrtc-1.0 not found", install the GStreamer WebRTC plugins
- Make sure port 8080 is not already in use
- Check browser console for WebRTC connection errors