# Implementation Details - Dora OpenAI WebSocket Server

## Code Structure

### Main Components

1. **WebSocket Handler** (`handle_client`)
   - Manages WebSocket lifecycle
   - Routes messages between client and Dora nodes
   - Handles session configuration

2. **Audio Processing**
   - `convert_pcm16_to_f32`: Converts PCM16 to float32
   - `convert_f32_to_pcm16`: Converts float32 back to PCM16
   - Resampling pipelines for both directions

3. **Dora Integration**
   - Dynamic dataflow spawning based on client config
   - Event routing between Dora nodes and WebSocket

## Key Functions

### Audio Conversion
```rust
fn convert_pcm16_to_f32(bytes: &[u8]) -> Vec<f32> {
    let mut samples = Vec::with_capacity(bytes.len() / 2);
    for chunk in bytes.chunks_exact(2) {
        let pcm16_sample = i16::from_le_bytes([chunk[0], chunk[1]]);
        let f32_sample = pcm16_sample as f32 / 32767.0;
        samples.push(f32_sample);
    }
    samples
}

fn convert_f32_to_pcm16(samples: &[f32]) -> Vec<u8> {
    let mut pcm16_bytes = Vec::with_capacity(samples.len() * 2);
    for &sample in samples {
        let clamped = sample.max(-1.0).min(1.0);
        let pcm16_sample = (clamped * 32767.0) as i16;
        pcm16_bytes.extend_from_slice(&pcm16_sample.to_le_bytes());
    }
    pcm16_bytes
}
```

### Resampling Pipeline

#### Initialization (line 408-429)
```rust
// Create downsampler for microphone input
const DOWNSAMPLE_CHUNK_SIZE: usize = 4800;
let mut downsampler = SincFixedIn::<f32>::new(...);
let mut audio_buffer = Vec::new();
```

#### Microphone Processing (line 609-641)
```rust
// Buffer and process in fixed chunks
audio_buffer.extend_from_slice(&f32_data);
while audio_buffer.len() >= DOWNSAMPLE_CHUNK_SIZE {
    let chunk: Vec<f32> = audio_buffer.drain(..DOWNSAMPLE_CHUNK_SIZE).collect();
    // Resample and send
}
```

#### TTS Processing (line 532-568)
```rust
// Process immediately for low latency
let mut resampler = SincFixedIn::<f32>::new(
    24000.0 / 32000.0,
    2.0,
    params,
    audio_data.len(),  // Exact size for this chunk
    1,
);
```

## Message Flow

### Client → Server → Dora
1. Client sends `InputAudioBufferAppend` with base64 PCM16 audio
2. Server decodes base64 → PCM16 → float32
3. Server buffers and resamples 24kHz → 16kHz
4. Server sends to speech-monitor via Dora

### Dora → Server → Client
1. PrimeSpeech sends 32kHz float32 audio
2. Server resamples 32kHz → 24kHz
3. Server converts float32 → PCM16 → base64
4. Server sends `ResponseAudioDelta` to client

## Configuration

### Environment Variables
- None required for resampling
- Dora node configuration handled by dataflow YAML

### Compile-time Constants
```rust
const DOWNSAMPLE_CHUNK_SIZE: usize = 4800;  // 200ms at 24kHz
```

### Resampler Parameters

**High Quality (Microphone)**:
- `sinc_len`: 256 - Longer filter for better quality
- `interpolation`: Cubic - Best quality interpolation
- `oversampling_factor`: 256 - High oversampling
- `window`: BlackmanHarris2 - Best stopband attenuation

**Fast (TTS)**:
- `sinc_len`: 64 - Shorter for lower latency
- `interpolation`: Linear - Faster processing
- `oversampling_factor`: 128 - Moderate oversampling
- `window`: Blackman - Good balance

## Memory Management

### Buffer Strategy
- **Microphone**: Accumulating buffer, drained in chunks
- **TTS**: Direct processing, no buffering
- **Vector Operations**: Use `drain()` for efficient memory reuse

### Performance Considerations
1. Pre-allocate vectors with `Vec::with_capacity()`
2. Reuse downsampler for all microphone chunks
3. Clone only when necessary (TTS output)
4. Use references where possible

## Error Handling

### Resampling Errors
```rust
.expect("Resampling failed")  // Panics on error
```
Consider changing to proper error propagation in production.

### WebSocket Errors
- Connection errors logged and handled gracefully
- Malformed messages skip processing with continue

## Debugging

### Logging Points
- Session configuration
- Chunk sizes
- Resampling operations (currently commented out)
- Error conditions

### Common Issues
1. **"Insufficient buffer size"**: SincFixedIn needs exact size
2. **No audio output**: Check buffering logic
3. **Poor quality**: Verify resampler parameters
4. **High latency**: Check buffer sizes and processing frequency

## Future Improvements

1. **Error Recovery**: Better error handling instead of panics
2. **Dynamic Quality**: Adjust resampling quality based on CPU load
3. **Metrics**: Add performance monitoring
4. **Buffer Optimization**: Adaptive buffer sizes based on network conditions
5. **Parallel Processing**: Process multiple chunks concurrently