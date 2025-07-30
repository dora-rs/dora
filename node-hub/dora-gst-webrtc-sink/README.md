# dora-gst-webrtc-sink

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Rust](https://img.shields.io/badge/rust-%23000000.svg?style=flat&logo=rust&logoColor=white)](https://www.rust-lang.org/)

A WebRTC sink node for [dora-rs](https://github.com/dora-rs/dora) that streams video via WebRTC with built-in signaling server. Supports multiple video sources and multiple clients per source.

## Features

- 🎥 Stream RGB8 format video from dora-rs inputs
- 📡 Built-in WebRTC signaling server (no external server needed)
- 🔄 Real-time streaming with latest frame priority (buffer size 1)
- 📹 **Multiple video sources**: Handle multiple cameras/video streams simultaneously
- 👥 **Multiple clients per source**: Each video source can serve multiple WebRTC clients
- 🔧 Dynamic video source management
- ⚙️ Configurable signaling server port via environment variable

## Prerequisites

- Rust 1.70+
- GStreamer 1.16+ with WebRTC plugins
- [dora-rs](https://github.com/dora-rs/dora)

### Installing GStreamer

#### Ubuntu/Debian
```bash
sudo apt-get update
sudo apt-get install \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav
```

#### macOS
```bash
brew install gstreamer gst-plugins-base gst-plugins-good gst-plugins-bad gst-plugins-ugly
```

#### Windows
Download and install from [GStreamer official website](https://gstreamer.freedesktop.org/download/)

## Installation

```bash
git clone https://github.com/yourusername/dora-gst-webrtc-sink.git
cd dora-gst-webrtc-sink
cargo build --release
```

## Usage

### Quick Start

1. Run the example demo:
```bash
cd example
./run_demo.sh
```

2. Open `example/webrtc-viewer.html` in your web browser
3. The viewer will automatically connect to both camera streams

### Environment Variables

- `SIGNALING_PORT`: WebSocket signaling server port (default: 8080)
- `RUST_LOG`: Log level (default: info)

### Input Format

The node supports two input formats:

1. **Legacy format** (backward compatible):
   - Input ID: `image`
   - Maps to the default video source

2. **Multi-camera format**:
   - Input ID: `<video_id>/frame`
   - Example: `camera1/frame`, `camera2/frame`, `front_camera/frame`
   - Each unique `video_id` creates a separate video stream

All inputs expect:
- **Format**: RGB8
- **Resolution**: Automatically detected from input frame size
- **Framerate**: Automatically detected from input timing
- **Encoding parameter**: "rgb8"

### WebRTC Client Connection

Connect to the WebSocket signaling server at:
```
ws://localhost:8080/<video_id>
```

Example URLs:
- `ws://localhost:8080/camera1` - First camera
- `ws://localhost:8080/camera2` - Second camera
- `ws://localhost:8080/default` - Legacy support

## Example Dataflow Configuration

### Single Camera (Legacy)
```yaml
nodes:
  - id: camera
    path: camera-node
    outputs:
      - image
  
  - id: webrtc-sink
    path: dora-gst-webrtc-sink
    inputs:
      image: camera/image
```

### Multiple Cameras
```yaml
nodes:
  - id: camera1
    path: camera-node
    outputs:
      - frame
  
  - id: camera2
    path: camera-node
    outputs:
      - frame
  
  - id: webrtc-sink
    path: dora-gst-webrtc-sink
    inputs:
      camera1/frame: camera1/frame
      camera2/frame: camera2/frame
```

## Web Client

The included `webrtc-viewer.html` provides:
- Automatic connection to multiple video streams
- Dynamic addition/removal of video streams
- Individual connection control per stream
- Real-time status and logging

## Architecture

```
┌─────────────┐     ┌──────────────────┐     ┌─────────────┐
│ Camera Node │────▶│ dora-gst-webrtc  │────▶│ Web Browser │
│   (RGB8)    │     │      -sink       │     │  (WebRTC)   │
└─────────────┘     │                  │     └─────────────┘
                    │  ┌─────────────┐ │
┌─────────────┐     │  │  Signaling  │ │     ┌─────────────┐
│ Camera Node │────▶│  │   Server    │ │────▶│ Web Browser │
│   (RGB8)    │     │  └─────────────┘ │     │  (WebRTC)   │
└─────────────┘     └──────────────────┘     └─────────────┘
```

## Development

### Building from Source
```bash
cargo build --release
```

### Running Tests
```bash
cargo test
```

### Code Structure
- `src/main.rs` - Main entry point and dora node integration
- `src/webrtc_server.rs` - WebRTC server implementation
- `src/peer_connection.rs` - Individual peer connection management
- `src/video_source_manager.rs` - Multiple video source management
- `src/signaling.rs` - WebRTC signaling protocol messages

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- [dora-rs](https://github.com/dora-rs/dora) - Dataflow-oriented robotics architecture
- [GStreamer](https://gstreamer.freedesktop.org/) - Multimedia framework
- [webrtcbin](https://gstreamer.freedesktop.org/documentation/webrtc/index.html) - GStreamer WebRTC implementation

## Troubleshooting

### GStreamer WebRTC plugin not found
Ensure GStreamer plugins are properly installed:
```bash
gst-inspect-1.0 webrtcbin
```

### Connection issues
1. Check firewall settings for port 8080 (signaling)
2. Ensure STUN server is accessible (uses Google's public STUN by default)
3. Check browser console for WebRTC errors

### Performance issues
- Adjust VP8 encoding parameters in `create_pipeline()`
- Consider reducing resolution or framerate
- Check CPU usage during encoding

## Roadmap

- [ ] H.264 codec support for better compatibility
- [ ] TURN server support for NAT traversal
- [ ] Connection quality metrics
- [ ] Support for additional video formats (NV12, I420)

## Contact

For questions, issues, or contributions, please use the GitHub issue tracker.
