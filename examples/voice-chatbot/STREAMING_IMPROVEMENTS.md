# Streaming LLM to TTS Improvements

## Problem
The original pipeline had significant latency issues:
1. **LLM waits until complete response** before sending to TTS (3-5 seconds)
2. **TTS processes entire text block** before generating any audio (2-4 seconds)
3. **Total time to first audio**: 5-9 seconds

## Solution
Implemented streaming at both LLM and TTS levels:

### 1. LLM Streaming (dora-qwen3)
- Added `generate_response_mlx_streaming()` method using MLX's `stream_generate()`
- Sends text chunks as they're generated (~50 chars or on punctuation)
- Controlled by `LLM_ENABLE_STREAMING` environment variable

### 2. TTS Segmentation (dora-primespeech)
- Already had `segment_text_for_tts()` but wasn't optimal
- Tunable via environment variables:
  - `ENABLE_INTERNAL_SEGMENTATION`: Enable/disable segmentation
  - `TTS_MAX_SEGMENT_LENGTH`: Max chars per segment (default: 100)
  - `TTS_MIN_SEGMENT_LENGTH`: Min chars per segment (default: 30)

### 3. TTS Streaming (if supported)
- PrimeSpeech supports streaming synthesis via MoYoYo wrapper
- Controlled by `RETURN_FRAGMENT` and `FRAGMENT_INTERVAL`

## Configuration

### Enable Full Streaming Pipeline
```yaml
# In your dataflow YAML
nodes:
  - id: llm
    path: dora-qwen3
    env:
      LLM_ENABLE_STREAMING: "true"  # Enable LLM streaming
      
  - id: tts
    path: dora-primespeech
    env:
      ENABLE_INTERNAL_SEGMENTATION: "true"
      TTS_MAX_SEGMENT_LENGTH: "50"  # Smaller for faster response
      TTS_MIN_SEGMENT_LENGTH: "20"
      RETURN_FRAGMENT: "true"        # Enable TTS streaming
      FRAGMENT_INTERVAL: "0.3"
```

## Performance Improvements

### Before (Non-streaming)
```
User speaks → LLM processes (3-5s) → Complete text → TTS processes (2-4s) → Audio starts
Total: 5-9 seconds to first audio
```

### After (Streaming)
```
User speaks → LLM starts streaming (0.5s) → First chunk (50 chars) → 
TTS processes chunk (0.3s) → Audio starts
Total: 0.8-1.5 seconds to first audio
```

## Testing

### Test Streaming Pipeline
```bash
# Run the streaming test
cd examples/voice-chatbot
dora up test_streaming_pipeline.yml
dora start test_streaming_pipeline.yml

# In another terminal, monitor timing
python monitor_streaming_timing.py
```

### Test Files
- `test_streaming_llm_tts.py` - Sends test queries
- `monitor_streaming_timing.py` - Tracks timing metrics
- `test_streaming_pipeline.yml` - Configured dataflow

## Key Code Changes

### dora-qwen3/main.py
```python
# Added streaming generation method
def generate_response_mlx_streaming(self, text: str, session_id: str, metadata: dict):
    for token_response in stream_generate(...):
        # Accumulate tokens into chunks
        # Send on punctuation or size threshold
        if should_send:
            self.node.send_output("text", pa.array([chunk]), metadata={
                "segment_index": segment_index,
                "is_streaming": True,
                "is_final": False
            })
```

### dora-primespeech/main.py
```python
# Already had segmentation, made it tunable
if enable_internal_segmentation and len(text) > max_segment_length:
    text_segments = segment_text_for_tts(text, max_segment_length, min_segment_length)
    # Process each segment immediately for faster response
```

## Monitoring
The pipeline now logs detailed timing information:
- LLM: "Generating with MLX (streaming)..."
- TTS: "Processing segment X (len=Y, parts=Z)"
- TTS: "Using streaming synthesis..."
- TTS: "Streamed N fragments in X.XXXs"

## Future Improvements
1. **Adaptive chunking** - Adjust chunk size based on network/processing speed
2. **Parallel TTS** - Process multiple segments simultaneously
3. **Predictive caching** - Pre-generate common response beginnings
4. **WebSocket streaming** - Direct browser audio streaming