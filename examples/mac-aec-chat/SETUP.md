# MAC-AEC Chat Setup Guide

## Quick Setup for New Installation

When installing on a new machine, you may encounter this error:
```
FileNotFoundError: MAC-AEC native library not found. Expected at: .../lib/libAudioCapture.dylib
```

## Solution

The native library `libAudioCapture.dylib` is required for macOS audio capture with echo cancellation.

### Option 1: Use Bundled Library
The library is included in the `lib/` directory:
```bash
# Verify it exists
ls lib/libAudioCapture.dylib

# If missing, it may not have been copied properly
```

### Option 2: Copy from Node
If the library is missing, copy it from the dora-mac-aec node:
```bash
# Create lib directory
mkdir -p lib

# Copy from dora-mac-aec node (if available)
cp ../../node-hub/dora-mac-aec/dora_mac_aec/lib/libAudioCapture.dylib lib/

# Or download/copy from another source
```

### Option 3: Build from Source
The library source is available in the VoiceDialogue repository:
- Repository: https://github.com/username/VoiceDialogue
- Path: `third_party/AECAudioRecorder/`

## File Structure
```
mac-aec-chat/
├── lib/
│   └── libAudioCapture.dylib  # Native library (REQUIRED)
├── mac_aec_simple_segmentation.py
├── mac-aec-chat.yml
└── README.md
```

## Dependencies

Install Python packages:
```bash
pip install dora-rs numpy pyarrow sounddevice
```

## Permissions

macOS requires microphone permission:
1. Run the script
2. Grant microphone access when prompted
3. Or manually enable in System Preferences → Security & Privacy → Privacy → Microphone

## Testing

Test the setup:
```bash
python mac_aec_simple_segmentation.py
```

You should see:
```
Starting MAC-AEC Simple Segmentation...
✓ Using bundled library: .../lib/libAudioCapture.dylib
MAC-AEC initialized successfully
Starting audio capture...
```

If successful, speak into your microphone and you should see speech detection messages.