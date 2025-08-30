# 🎙️ Voice Assistant Pipeline Architecture

## Overview

This is a complete voice assistant pipeline for macOS using Dora's dataflow architecture. It provides real-time speech interaction with acoustic echo cancellation (AEC), automatic speech recognition (ASR), large language model (LLM) processing, and text-to-speech (TTS) synthesis.

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         Audio Input Layer                        │
├─────────────────────────────────────────────────────────────────┤
│  MAC-AEC (macOS Audio Echo Cancellation)                        │
│  • Captures microphone input                                    │
│  • Removes speaker echo                                         │
│  • Voice Activity Detection (VAD)                               │
│  • Audio segmentation                                           │
└────────────┬────────────────────────────────────────────────────┘
             │ audio_segment
             ↓
┌─────────────────────────────────────────────────────────────────┐
│                      Speech Recognition Layer                    │
├─────────────────────────────────────────────────────────────────┤
│  ASR (Automatic Speech Recognition)                             │
│  • FunASR engine for Chinese                                    │
│  • Whisper for multilingual                                     │
│  • Real-time transcription                                      │
│  • Language detection                                           │
└────────────┬────────────────────────────────────────────────────┘
             │ transcription
             ↓
┌─────────────────────────────────────────────────────────────────┐
│                    Language Processing Layer                     │
├─────────────────────────────────────────────────────────────────┤
│  Qwen3 LLM                                                      │
│  • MLX optimized for Apple Silicon                              │
│  • Streaming response generation                                │
│  • Context management                                           │
│  • Token-based history                                          │
└────────────┬────────────────────────────────────────────────────┘
             │ text (streaming chunks)
             ↓
┌─────────────────────────────────────────────────────────────────┐
│                      Text Processing Layer                       │
├─────────────────────────────────────────────────────────────────┤
│  Text Segmenter                                                 │
│  • Queue-based buffering                                        │
│  • Intelligent chunking                                         │
│  • Backpressure control                                         │
│  • Punctuation filtering                                        │
└────────────┬────────────────────────────────────────────────────┘
             │ text_segment
             ↓
┌─────────────────────────────────────────────────────────────────┐
│                      Speech Synthesis Layer                      │
├─────────────────────────────────────────────────────────────────┤
│  PrimeSpeech TTS                                                │
│  • Multiple voice options                                       │
│  • Real-time synthesis                                          │
│  • Segment completion signals                                   │
└────────────┬────────────────────────────────────────────────────┘
             │ audio
             ↓
┌─────────────────────────────────────────────────────────────────┐
│                        Audio Output Layer                        │
├─────────────────────────────────────────────────────────────────┤
│  Audio Player                                                   │
│  • macOS audio output                                           │
│  • Buffer management                                            │
│  • Playback control                                             │
└─────────────────────────────────────────────────────────────────┘
```

## Component Details

### 1. MAC-AEC (macOS Audio Echo Cancellation)
- **Type**: Dynamic node (`mac_aec_simple_segmentation.py`)
- **Function**: Captures microphone input with echo cancellation
- **Critical Implementation**:
  - **MUST drain audio buffer completely** (loop until `get_audio_data()` returns None)
  - **Poll every 10ms** (not 33ms) to avoid missing audio
  - **Send ALL frames**, not just samples (was losing 97% of audio!)
  - **Uses `dora-aec` library** (NOT `dora-mac-aec` which lacks proper AEC)
  - **Format conversion**: int16 bytes → float32 arrays for ASR
- **Outputs**:
  - `audio`: Continuous audio stream (ALL frames)
  - `is_speaking`: Boolean speech detection
  - `speech_started`: Event when speech begins
  - `speech_ended`: Event when speech ends
  - `audio_segment`: Segmented audio for ASR (float32)

### 2. ASR (Automatic Speech Recognition)
- **Path**: `../../node-hub/dora-asr`
- **Engine**: FunASR (default) or Whisper
- **Configuration**:
  - Language: Chinese (`zh`) or auto-detect
  - Punctuation restoration enabled
  - Confidence scoring available
- **Queue Size**: 10 segments buffered

### 3. Qwen3 LLM
- **Path**: `../../node-hub/dora-qwen3`
- **Model**: Qwen3-32B-MLX-6bit (configurable)
- **Features**:
  - MLX acceleration on Apple Silicon
  - Streaming output for fast response
  - Token-based history management (3000 tokens)
  - Configurable temperature and max tokens

### 4. Text Segmenter
- **Path**: `../../node-hub/dora-text-segmenter/dora_text_segmenter/queue_based_segmenter.py`
- **Function**: Buffers LLM output and sends to TTS one segment at a time
- **Features**:
  - No deadlock - first segment sent immediately
  - Skips punctuation-only segments
  - Queue-based with backpressure control

### 5. PrimeSpeech TTS
- **Path**: `../../node-hub/dora-primespeech`
- **Voices**: Doubao, Luo Xiang, Yang Mi, Zhou Jielun, Ma Yun, Maple, Cove
- **Features**:
  - Chinese and English support
  - Internal text segmentation for long texts
  - Segment completion signals for flow control

### 6. Audio Player
- **Type**: Dynamic node
- **Function**: Plays synthesized audio through system speakers
- **Features**:
  - Buffer status reporting
  - Real-time playback

### 7. Viewer (Optional)
- **Type**: Dynamic node
- **Function**: Visual monitoring of the pipeline
- **Monitors**:
  - ASR transcriptions
  - LLM outputs
  - Text segments
  - Speech events

## Data Flow

### 1. Speech Input Flow
```
Microphone → MAC-AEC → VAD Detection → Audio Segmentation → ASR
```

### 2. Processing Flow
```
ASR Transcription → Qwen3 LLM → Streaming Text → Text Segmenter → TTS
```

### 3. Audio Output Flow
```
TTS Audio → Audio Player → System Speakers
```

### 4. Feedback Loop
```
TTS Completion → Text Segmenter → Next Segment
```

## Key Features

### Real-time Processing
- Streaming LLM responses begin before full generation
- TTS starts synthesizing before full text is available
- Pipeline processes segments in parallel

### Echo Cancellation
- Removes speaker output from microphone input
- Prevents feedback loops
- Enables hands-free operation

### Intelligent Segmentation
- VAD-based audio segmentation
- Text chunking for optimal TTS
- Queue management prevents data loss

### History Management
- Token-based conversation history
- Automatic trimming to stay within limits
- Context preservation across turns

## Configuration

### Model Selection
Models can be changed via environment variables:
- `MLX_MODEL`: LLM model (e.g., "Qwen/Qwen3-14B-MLX-4bit")
- `WHISPER_MODEL`: ASR model (e.g., "large-v3")
- `VOICE_NAME`: TTS voice selection

### Performance Tuning
- `MAX_TOKENS`: LLM response length
- `TEMPERATURE`: LLM creativity (0.0-1.0)
- `MAX_HISTORY_EXCHANGES`: Conversation memory
- `QUEUE_SIZE`: ASR buffer size

### Language Settings
- `LANGUAGE`: ASR language (zh/en/auto)
- `TEXT_LANG`: TTS language
- `ENABLE_PUNCTUATION`: Punctuation restoration

## System Requirements

### Hardware
- **macOS**: Required for MAC-AEC
- **Apple Silicon**: Recommended for MLX acceleration
- **RAM**: 16GB minimum, 32GB recommended
- **Storage**: 50GB for models

### Software
- **Dora**: Latest version
- **Python**: 3.8+
- **Dependencies**: See node-hub requirements

## Performance Characteristics

### Latency
- **First response**: 1-2 seconds
- **Speech detection**: <100ms
- **ASR processing**: 200-500ms
- **LLM first token**: 500ms-1s
- **TTS synthesis**: 100-300ms per segment

### Throughput
- **Audio**: 16kHz sampling rate
- **ASR**: Real-time factor ~0.3
- **LLM**: 20-50 tokens/second
- **TTS**: 2-3x real-time

### Resource Usage
- **CPU**: 20-40% average
- **GPU**: 60-80% during inference
- **RAM**: 8-12GB for models
- **Network**: Offline capable

## Error Handling

### Graceful Degradation
- Falls back to CPU if GPU unavailable
- Continues with partial transcriptions
- Skips corrupted audio segments

### Recovery Mechanisms
- Automatic reconnection on node failure
- Queue persistence prevents data loss
- History recovery from cache

## Monitoring

The pipeline provides comprehensive monitoring through:
- Log outputs from each node
- Visual viewer for real-time status
- Performance metrics in logs
- Buffer status indicators

## Critical Lessons Learned (Golden Version)

### Audio Capture Issues & Solutions

#### Problem 1: Lost 97% of Audio
- **Issue**: Only sending every 30th frame
- **Solution**: Send EVERY frame continuously
```python
# WRONG
if frame_count % 30 == 0:
    node.send_output("audio", ...)

# CORRECT
node.send_output("audio", pa.array(audio_frame, type=pa.float32()))
```

#### Problem 2: Buffer Not Drained
- **Issue**: Only getting one chunk per cycle, missing buffered audio
- **Solution**: Drain ALL available chunks
```python
# WRONG
audio_data, vad = aec.get_audio_data()

# CORRECT
all_audio = []
while True:
    audio_data, vad = aec.get_audio_data()
    if audio_data is None:
        break
    all_audio.append(audio_data)
```

#### Problem 3: Echo Not Cancelled
- **Issue**: Using `dora-mac-aec` which doesn't enable AEC properly
- **Solution**: Use `dora-aec` with proper VoiceProcessingIO implementation

#### Problem 4: Polling Too Slow
- **Issue**: 33ms intervals created gaps in audio
- **Solution**: Poll every 10ms for continuous capture

### VAD Segmentation Parameters
- **Speech start**: 3 consecutive voice frames
- **Speech end**: 10 consecutive silence frames  
- **Min segment**: 0.3 seconds (4800 samples)
- **Max segment**: 10 seconds (160000 samples)

### Audio Format Pipeline
```
Native Library → int16 bytes
    ↓ np.frombuffer(dtype=np.int16)
Numpy int16 array
    ↓ .astype(np.float32) / 32768.0
Float32 array [-1.0, 1.0]
    ↓ pa.array(type=pa.float32())
PyArrow array → ASR
```

## Extension Points

### Custom Models
- Replace ASR with custom engines
- Use different LLM models
- Add custom TTS voices

### Additional Processing
- Add translation nodes
- Insert sentiment analysis
- Include custom filters

### Integration
- Connect to external services
- Add database logging
- Implement custom protocols