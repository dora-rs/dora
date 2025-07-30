#!/bin/bash

echo "=== dora-gst-webrtc-sink Demo ==="
echo

# Check if dora is installed
if ! command -v dora &> /dev/null; then
    echo "Error: 'dora' command not found. Please install dora-rs first."
    echo "Visit: https://github.com/dora-rs/dora"
    exit 1
fi

# Check if GStreamer WebRTC plugin is installed
if ! command -v gst-inspect-1.0 webrtc &> /dev/null; then
    echo "Error: GStreamer not found. Please install GStreamer and its WebRTC plugins."
    echo "Ubuntu/Debian: sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-bad1.0-dev"
    exit 1
fi

echo
echo "Building dataflow..."
dora build dataflow.yml
if [ $? -ne 0 ]; then
    echo "Error: Build failed. Please check the error messages above."
    exit 1
fi

echo
echo "Starting multi-camera WebRTC streaming demo..."
echo "WebRTC signaling server will be available at: ws://localhost:8080/<video_id>"
echo
echo "Available video streams:"
echo "  - camera1: Moving ball pattern (ws://localhost:8080/camera1)"
echo "  - camera2: SMPTE test pattern (ws://localhost:8080/camera2)"
echo
echo "To view the streams:"
echo "1. Open 'webrtc-viewer.html' in a web browser"
echo "2. Both cameras will connect automatically"
echo "3. You can add more video streams using the interface"
echo
echo "Press Ctrl+C to stop"
echo

# Start the dataflow
dora run dataflow.yml