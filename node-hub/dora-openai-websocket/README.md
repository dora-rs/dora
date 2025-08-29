# Dora OpenAI WebSocket Server

A WebSocket server that bridges OpenAI's Realtime API format with Dora's audio processing nodes, providing real-time audio resampling for ASR and TTS.

## Overview

This server handles:
- WebSocket connections from OpenAI Realtime API clients
- Audio downsampling from 24kHz to 16kHz for ASR/speech-monitor
- Audio upsampling from 32kHz to 24kHz for TTS output
- Automatic Dora dataflow orchestration

## Audio Resampling Implementation

### Key Design Decisions

1. **Microphone Input (24kHz → 16kHz)**
   - Uses buffering with a single pre-created resampler
   - Optimized for quality over latency
   - Processes in fixed 4800-sample chunks (200ms at 24kHz)

2. **TTS Output (32kHz → 24kHz)**
   - Creates resampler per chunk for immediate processing
   - Optimized for low latency
   - Handles variable chunk sizes from PrimeSpeech

### Technical Details

#### Downsampling (Microphone → ASR)
```rust
// Single high-quality resampler created at startup
const DOWNSAMPLE_CHUNK_SIZE: usize = 4800; // 200ms at 24kHz

let resampler_params = SincInterpolationParameters {
    sinc_len: 256,          // High quality filter
    f_cutoff: 0.95,         // Cutoff frequency ratio
    interpolation: SincInterpolationType::Cubic,  // High quality
    oversampling_factor: 256,  // High oversampling
    window: WindowFunction::BlackmanHarris2,  // Best stopband attenuation
};

let mut downsampler = SincFixedIn::<f32>::new(
    16000.0 / 24000.0,  // Resample ratio (2/3)
    2.0,                // Max delay
    resampler_params,
    DOWNSAMPLE_CHUNK_SIZE,  // Fixed input size
    1,                  // 1 channel (mono)
);

// Buffer incoming audio and process in fixed chunks
audio_buffer.extend_from_slice(&f32_data);
while audio_buffer.len() >= DOWNSAMPLE_CHUNK_SIZE {
    let chunk: Vec<f32> = audio_buffer.drain(..DOWNSAMPLE_CHUNK_SIZE).collect();
    let input = vec![chunk];
    let output = downsampler.process(&input, None)?;
    // Send to speech-monitor
}
```

#### Upsampling (TTS → Client)
```rust
// Create resampler per chunk for low latency
let params = SincInterpolationParameters {
    sinc_len: 64,  // Lower for faster processing
    f_cutoff: 0.95,
    interpolation: SincInterpolationType::Linear,  // Faster
    oversampling_factor: 128,
    window: WindowFunction::Blackman,
};

let mut resampler = SincFixedIn::<f32>::new(
    24000.0 / 32000.0,  // Resample ratio (3/4)
    2.0,
    params,
    audio_data.len(),   // Exact input size for this chunk
    1,
);

let input = vec![audio_data];
let output = resampler.process(&input, None)?;
```

## Sample Rate Requirements

- **OpenAI Realtime API**: 24kHz PCM16 (fixed, cannot be changed)
- **FunASR/Speech-monitor**: 16kHz only
- **PrimeSpeech TTS**: 32kHz output by default
- **WebSocket chunks**: Variable size (typically 512-2048 samples)

## Dependencies

```toml
[dependencies]
rubato = "0.14"  # High-quality audio resampling
fastwebsockets = "0.8"
dora-node-api = { workspace = true }
# ... other dependencies
```

## Building

```bash
cargo build --release
```

## Usage

The server is automatically started by Dora when needed. It:
1. Listens on port 8080 for WebSocket connections
2. Spawns appropriate Dora dataflows based on client configuration
3. Routes audio between WebSocket and Dora nodes with proper resampling

## Architecture

```
┌─────────────┐     24kHz PCM16     ┌──────────────┐
│   Client    │◄────────────────────►│   WebSocket  │
│  (Browser)  │                      │    Server    │
└─────────────┘                      └──────┬───────┘
                                            │
                                    ┌───────┴────────┐
                                    │                │
                              16kHz │                │ 32kHz
                                    ▼                ▼
                            ┌──────────────┐  ┌──────────────┐
                            │Speech-Monitor│  │  PrimeSpeech │
                            │     +ASR     │  │     (TTS)    │
                            └──────────────┘  └──────────────┘
```

## Performance Optimizations

1. **Resampler Reuse**: Microphone downsampler created once, reused for all chunks
2. **Buffering**: Accumulates variable WebSocket chunks into fixed processing sizes
3. **Parameter Tuning**: Different quality settings for downsampling (quality) vs upsampling (speed)
4. **Memory Management**: Efficient buffer draining and vector operations

## Known Issues and Solutions

### Issue: SincFixedIn requires exact input sizes
**Solution**: Use buffering to accumulate exact chunk sizes for microphone input

### Issue: Creating resamplers is expensive
**Solution**: Create once and reuse for microphone; accept per-chunk creation for TTS due to latency requirements

### Issue: Variable WebSocket chunk sizes
**Solution**: Buffer for microphone input; direct processing for TTS output

## Testing

Test the resampling quality using the wav-player-downsample example:
```bash
cd examples/wav-player-downsample
./run_rust_test.sh test24.wav
```

## Version History

See [RESAMPLING_HISTORY.md](./RESAMPLING_HISTORY.md) for detailed implementation history and failed attempts.