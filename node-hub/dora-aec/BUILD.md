# Building the AEC Native Library

## Prerequisites

- macOS with Xcode Command Line Tools
- Swift 5.0 or later

## Building from Source

The AEC node uses a native macOS library (`libAudioCapture.dylib`) for acoustic echo cancellation using the VoiceProcessingIO AudioUnit.

### Compile the Swift Library

1. Navigate to the Swift source directory:
```bash
cd /Users/yuechen/home/VoiceDialogue/third_party/AECAudioRecorder
```

2. Compile the Swift source into a dynamic library:
```bash
swiftc -emit-library -o libAudioCapture.dylib AECAudioStream.swift
```

3. Copy the compiled library to the dora-aec package:
```bash
cp libAudioCapture.dylib /Users/yuechen/home/conversation/dora/node-hub/dora-aec/dora_aec/lib/
```

## Library Interface

The compiled library provides the following C interface:

- `startRecord()` - Start audio recording with AEC
- `stopRecord()` - Stop audio recording
- `getAudioData(int* size, bool* isVoiceActive)` - Get audio data with VAD status
- `freeAudioData(uint8_t* buffer)` - Free allocated audio buffer

## How It Works

1. **VoiceProcessingIO AudioUnit**: Uses macOS's built-in audio processing unit designed for voice communication
2. **Acoustic Echo Cancellation**: Hardware-accelerated AEC removes echo from speaker output
3. **Voice Activity Detection**: Built-in VAD detects when user is speaking
4. **Real-time Processing**: Low-latency audio processing suitable for real-time conversation

## Troubleshooting

### Permission Issues

If you get permission errors when running the compiled library, ensure:
1. Terminal/IDE has microphone access in System Settings → Privacy & Security → Microphone
2. The library has proper execution permissions: `chmod +x libAudioCapture.dylib`

### Architecture Mismatch

If you encounter architecture issues on Apple Silicon Macs:
```bash
# Compile for Apple Silicon (arm64)
swiftc -emit-library -target arm64-apple-macos -o libAudioCapture.dylib AECAudioStream.swift

# Compile for Intel (x86_64)
swiftc -emit-library -target x86_64-apple-macos -o libAudioCapture.dylib AECAudioStream.swift

# Universal binary (works on both)
swiftc -emit-library -o libAudioCapture_arm64.dylib -target arm64-apple-macos AECAudioStream.swift
swiftc -emit-library -o libAudioCapture_x86_64.dylib -target x86_64-apple-macos AECAudioStream.swift
lipo -create libAudioCapture_arm64.dylib libAudioCapture_x86_64.dylib -output libAudioCapture.dylib
```

## Alternative: Pre-compiled Library

If you don't want to compile from source, the pre-compiled library can be found at:
- VoiceDialogue project: `/Users/yuechen/home/VoiceDialogue/libraries/libAudioCapture.dylib`

Simply copy it to the dora-aec lib directory:
```bash
cp /Users/yuechen/home/VoiceDialogue/libraries/libAudioCapture.dylib \
   /Users/yuechen/home/conversation/dora/node-hub/dora-aec/dora_aec/lib/
```