# Voice Chatbot Implementation Changes

## Overview
This document details all changes made to implement a voice chatbot system by combining the realtime-interpreter and primespeech-streaming-test examples.

## Architecture
```
Microphone → Speech Monitor → ASR → Chat Controller → Qwen3 LLM
                ↓                           ↓
         question_ended                 (direct)
                                           ↓
Audio Player ← PrimeSpeech TTS ← Text Segmenter
     ↓
buffer_status → Chat Controller (monitors for resume)
```

## Major Components Added/Modified

### 1. Speech Monitor (`dora-speechmonitor`)
**Location**: `/Users/yuechen/home/conversation/dora/node-hub/dora-speechmonitor/`

#### Changes:
- **Added pause/resume functionality** (`main.py` lines 95-126)
  - New input: `control` for receiving pause/resume commands
  - When paused: Drops all incoming audio to prevent hearing system's own voice
  - Resets state on pause/resume for clean transitions

- **Added question_ended output** (`main.py` lines 289-321)
  - New output triggered after 3 seconds of silence following speech
  - Configurable via `QUESTION_END_SILENCE_MS` environment variable
  - Provides more definitive end-of-question signal than `speech_ended`
  - Tracks `last_speech_end_time` and `question_end_sent` flag

### 2. Chat Controller (NEW) (`dora-chat-controller`)
**Location**: `/Users/yuechen/home/conversation/dora/node-hub/dora-chat-controller/`

#### Created as static node package:
```python
# Main features (main.py):
- State management: listening, processing, answering
- Accumulates ASR transcriptions until question complete
- Monitors buffer_status to detect answer playback completion
- Sends pause/resume commands to Speech Monitor
- Maintains conversation history (configurable size)
```

#### Key Configuration:
```python
MIN_QUESTION_LENGTH = 5  # Minimum characters
MAX_CONVERSATION_HISTORY = 10  # Q&A pairs to remember
ANSWER_COMPLETE_BUFFER_THRESHOLD = 2.0  # Resume when buffer < 2%
BUFFER_CHECK_INTERVAL = 0.1  # Check every 100ms
```

### 3. Audio Player (`audio_player.py`)

#### Critical Bug Fixes:

1. **Buffer Underrun Handling** (lines 59-80)
   - **Issue**: Buffer reported having samples even when empty
   - **Fix**: Properly drain buffer to 0% when underrun occurs
   ```python
   if self.available_samples < num_samples:
       # Partial read - return what we have plus zeros
       actual_samples = self.available_samples
       # ... read available samples ...
       self.available_samples = 0  # Buffer is now empty
   ```

2. **Event Loop Timeout** (line 218)
   - **Issue**: 20-second timeout caused long delays in buffer reporting
   - **Fix**: Reduced to 1 second for regular status updates
   ```python
   event = node.next(timeout=1.0)  # Was: timeout=20
   ```

3. **Immediate Playback** (lines 295-300)
   - **Issue**: Waited for 3 seconds of buffered audio before playing
   - **Fix**: Start playback immediately when any audio arrives
   ```python
   if not playback_started and len(audio_data) > 0:
       player.resume()  # Start playing immediately
   ```

4. **Aggressive Status Reporting** (lines 312-344)
   - **Added**: More frequent updates when buffer < 10%
   - **Added**: Debug logging when buffer < 5%
   ```python
   if buffer_percentage < 10.0:
       status_check_interval = 0.05  # 50ms when low
   ```

### 4. Log Display Node (NEW) (`log_display.py`)
**Features**:
- Collects logs from all static nodes
- Color-coded by node type and log level
- Formatted timestamps (HH:MM:SS.mmm)
- Maintains buffer of last 50 logs
- Handles both JSON and plain text log formats

### 5. Dataflow Configuration (`voice_chatbot.yml`)

#### Key Settings:
```yaml
# Speech Monitor
QUESTION_END_SILENCE_MS: 3000  # 3s silence = definitive question end
USER_SILENCE_THRESHOLD_MS: 1500  # 1.5s for internal processing

# Chat Controller
ANSWER_COMPLETE_BUFFER_THRESHOLD: 2.0  # Resume when buffer < 2%
BUFFER_CHECK_INTERVAL: 0.1  # Check every 100ms

# ASR
ASR_ENGINE: auto  # Falls back to Whisper if FunASR fails
LANGUAGE: zh  # Chinese/English detection

# Qwen3 LLM
USE_MLX: auto  # Apple Silicon optimization
MLX_MAX_TOKENS: 256  # Keep responses concise for voice
```

### 6. FunASR Support
- **Issue**: ONNX export was failing
- **Fix**: Installed `funasr` package for ONNX model export
- **Fallback**: Auto mode falls back to Whisper if FunASR fails

## Latency Optimizations

### Total Latency Reduction: ~27 seconds → ~1-2 seconds

1. **Buffer Status Reporting**: 
   - Before: Only on audio events (could be 20+ seconds)
   - After: Every 1 second via timeout

2. **Buffer Empty Detection**:
   - Before: Buffer incorrectly reported >0% when empty
   - After: Correctly reports 0% immediately

3. **Playback Start**:
   - Before: Waited for 3 seconds of audio
   - After: Starts immediately

4. **Chat Controller Response**:
   - Before: 500ms rate limiting
   - After: 100ms checks when buffer < threshold

## Data Flow

1. **User speaks** → Microphone → Speech Monitor
2. **Speech detected** → `speech_started` event → Chat Controller starts accumulating
3. **Silence detected** → `question_ended` event (after 3s) → Chat Controller sends to LLM
4. **LLM responds** → Text directly to Text Segmenter → PrimeSpeech TTS → Audio Player
5. **Buffer monitoring** → Audio Player sends `buffer_status` every 1s
6. **Playback complete** → Buffer < 2% → Chat Controller resumes Speech Monitor
7. **Ready for next question**

## Critical Design Decisions

1. **Static vs Dynamic Nodes**:
   - Chat Controller: Static (for reliability)
   - Audio Player: Dynamic (needs audio hardware access)
   - Log Display: Dynamic (UI component)

2. **Direct Routing**:
   - LLM output goes directly to Text Segmenter (not through Chat Controller)
   - Simplifies flow and reduces latency

3. **Buffer Management**:
   - No water marks - play whatever is in buffer
   - Audio callback handles underruns gracefully

4. **Question Detection**:
   - Two-tier: `speech_ended` (500ms) and `question_ended` (3s)
   - More reliable question boundary detection

## Testing & Debugging

Use the log display to monitor:
```bash
python log_display.py  # In separate terminal
```

Key metrics to watch:
- Buffer percentage during playback
- Time between "Question ENDED" and "Speech Monitor RESUMED"
- Underrun count in audio player

## Known Issues & Future Improvements

1. **Sample Rate Mismatch**: Microphone at 16kHz, Audio Player at 32kHz (works but could be unified)
2. **FunASR ONNX Export**: First run takes time to export models
3. **Conversation History**: Currently in-memory only, could persist

## Configuration Files

- Main dataflow: `voice_chatbot.yml`
- Run script: `run_chatbot.sh`
- Dynamic nodes: `microphone_input.py`, `audio_player.py`, `log_display.py`
- Static nodes: `dora-speechmonitor`, `dora-chat-controller`, `dora-asr`, `dora-qwen3`, etc.