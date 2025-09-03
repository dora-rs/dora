# WebSocket Audio Resampling - Implementation History and Golden Version

## Final Working Solution (Golden Version)

### Overview
- **Microphone Input (24kHz → 16kHz)**: Uses buffering with a single resampler for high quality
- **TTS Output (32kHz → 24kHz)**: Creates resampler per chunk for low latency

### Key Components

#### 1. Microphone Downsampling (24kHz → 16kHz)
```rust
// Single resampler created at startup
const DOWNSAMPLE_CHUNK_SIZE: usize = 4800; // 200ms at 24kHz

let resampler_params = SincInterpolationParameters {
    sinc_len: 256,          // High quality filter
    f_cutoff: 0.95,
    interpolation: SincInterpolationType::Cubic,
    oversampling_factor: 256,
    window: WindowFunction::BlackmanHarris2,
};

let mut downsampler = SincFixedIn::<f32>::new(
    16000.0 / 24000.0,  // Resample ratio (2/3)
    2.0,                // Max delay
    resampler_params,
    DOWNSAMPLE_CHUNK_SIZE,
    1,                  // 1 channel (mono)
).expect("Failed to create downsampler");

let mut audio_buffer = Vec::new(); // Buffer for accumulating audio

// Process incoming audio
audio_buffer.extend_from_slice(&f32_data);
while audio_buffer.len() >= DOWNSAMPLE_CHUNK_SIZE {
    let chunk: Vec<f32> = audio_buffer.drain(..DOWNSAMPLE_CHUNK_SIZE).collect();
    let input = vec![chunk];
    let output = downsampler.process(&input, None).expect("Resampling failed");
    let f32_data = output[0].clone();
    // Send to speech-monitor
}
```

#### 2. TTS Upsampling (32kHz → 24kHz)
```rust
// Create resampler per chunk for low latency
let params = SincInterpolationParameters {
    sinc_len: 64,  // Lower for faster processing
    f_cutoff: 0.95,
    interpolation: SincInterpolationType::Linear,
    oversampling_factor: 128,
    window: WindowFunction::Blackman,
};

let mut resampler = SincFixedIn::<f32>::new(
    24000.0 / 32000.0,  // Resample ratio (3/4)
    2.0,
    params,
    audio_data.len(),   // Exact input size
    1,
).expect("Failed to create TTS resampler");

let input = vec![audio_data];
let output = resampler.process(&input, None).expect("TTS resampling failed");
let resampled = output[0].clone();
```

## Implementation History

### Failed Attempts

#### 1. Simple Sample Dropping (FAILED)
```rust
// Just took 2 out of every 3 samples
let f32_data = f32_data.into_iter()
    .enumerate()
    .filter(|(i, _)| i % 3 < 2)
    .map(|(_, v)| v)
    .collect();
```
**Issue**: Severe aliasing, poor ASR accuracy, distorted audio

#### 2. Linear Interpolation (FAILED)
```rust
// Simple linear interpolation
for i in 0..output_len {
    let src_pos = i as f32 * 24000.0 / 16000.0;
    let src_idx = src_pos as usize;
    let frac = src_pos - src_idx as f32;
    let sample = f32_data[src_idx] * (1.0 - frac) + f32_data[src_idx + 1] * frac;
    f32_resampled.push(sample);
}
```
**Issue**: Insufficient quality for ASR accuracy

#### 3. SincFixedOut with Fixed Output Size (FAILED)
```rust
let mut downsampler = SincFixedOut::<f32>::new(
    16000.0 / 24000.0,
    2.0,
    resampler_params,
    1024,  // Fixed output size
    1,
).expect("Failed to create downsampler");
```
**Issue**: "Insufficient buffer size" - SincFixedOut expects specific input size to produce exact output

#### 4. SincFixedIn with Single Fixed Size (FAILED)
```rust
let mut resampler = SincFixedIn::<f32>::new(
    16000.0 / 24000.0,
    2.0,
    params,
    4800,  // Fixed expected size
    1,
).expect("Failed to create resampler");
```
**Issue**: "Insufficient buffer size 512 for input channel 0, expected 4800" - needs EXACT size

#### 5. SincFixedIn with Large Max Size (FAILED)
```rust
let mut resampler = SincFixedIn::<f32>::new(
    16000.0 / 24000.0,
    2.0,
    params,
    48000,  // Attempted "max" size
    1,
).expect("Failed to create resampler");
```
**Issue**: "Insufficient buffer size 512 for input channel 0, expected 48000" - not a max, but exact requirement

#### 6. Create Resampler Per Chunk (PARTIALLY WORKED)
```rust
// For each audio chunk
let mut resampler = SincFixedIn::<f32>::new(
    16000.0 / 24000.0,
    2.0,
    params,
    f32_data.len(),  // Exact chunk size
    1,
).expect("Failed to create resampler");
```
**Issue**: Worked but too slow, caused dataflow to get stuck due to object creation overhead

### Key Learnings

1. **SincFixedIn requires EXACT input size** - not "up to" or "at least", but exactly the size specified
2. **Creating resamplers is expensive** - should be done once, not per chunk
3. **WebSocket chunks are variable sized** - need buffering to create fixed-size chunks
4. **Simple algorithms insufficient for ASR** - need proper anti-aliasing filters
5. **TTS needs low latency** - can trade some efficiency for immediate processing

### Why the Final Solution Works

1. **Microphone Input Buffering**:
   - Accumulates variable WebSocket chunks (512, 1024, etc.)
   - Processes in fixed 4800-sample chunks
   - Single resampler created once = efficient
   - High quality parameters = good ASR accuracy

2. **TTS Direct Processing**:
   - Processes immediately without buffering = low latency
   - Creates resampler per chunk = handles variable sizes
   - Optimized parameters (smaller sinc_len) = acceptable performance
   - Direct frame sending = no buffering delays

### Configuration Notes

- **Sample Rates**:
  - OpenAI Realtime API: 24kHz PCM16 (fixed requirement)
  - FunASR/Speech-monitor: 16kHz only
  - PrimeSpeech TTS: 32kHz output by default

- **Chunk Sizes**:
  - Microphone buffer: 4800 samples (200ms at 24kHz)
  - WebSocket chunks: Variable (typically 512-2048 samples)

### Dependencies
```toml
rubato = "0.14"  # High-quality audio resampling library
```

## Testing
The solution was validated with:
1. WAV file test player (confirmed algorithm quality)
2. Real-time WebSocket with microphone input
3. TTS voice generation and playback

All three scenarios work correctly with good audio quality and ASR accuracy.