# Text Segmenter Node

Intelligent text segmentation for streaming TTS systems. Breaks LLM responses into optimal chunks for natural speech synthesis with backpressure control.

## Overview

The Text Segmenter receives streaming text from LLMs and intelligently breaks it into segments suitable for TTS processing. It implements backpressure control to prevent buffer overflow and ensures smooth, natural-sounding speech output.

## Features

- **Smart Segmentation**: Breaks text at natural boundaries (sentences, phrases)
- **Streaming Support**: Processes text as it arrives from LLM
- **Backpressure Control**: Pauses/resumes based on audio buffer status
- **Punctuation Detection**: Uses NLP for intelligent break points
- **Configurable Lengths**: Adjustable min/max segment sizes
- **Buffer Management**: Coordinates with TTS completion signals

## Installation

```bash
pip install -e dora-text-segmenter
```

## Inputs

| Input | Type | Description |
|-------|------|-------------|
| `text` | String | Streaming text from LLM |
| `control_signal` | String | Pause/resume from conversation-controller |
| `control` | String | External control commands |
| `tts_complete` | Signal | Segment processing complete from TTS |

## Outputs

| Output | Type | Description |
|--------|------|-------------|
| `text_segment` | String | Segmented text chunks for TTS |
| `status` | Dict | Segmentation statistics and state |

## Configuration

```yaml
env:
  # Segmentation parameters
  MAX_SEGMENT_LENGTH: 50        # Maximum characters per segment
  MIN_SEGMENT_LENGTH: 10        # Minimum characters per segment
  
  # Backpressure control
  USE_BACKPRESSURE_CONTROL: true  # Enable buffer-based flow control
  
  # Testing
  USE_TEST_TEXT: false          # Use test text instead of LLM input
  
  # Logging
  LOG_LEVEL: INFO              # DEBUG, INFO, WARNING, ERROR
```

## Segmentation Algorithm

```python
1. Accumulate incoming text in buffer
2. When buffer exceeds MIN_SEGMENT_LENGTH:
   a. Look for sentence boundary (. ! ?)
   b. If none, look for phrase boundary (, ; :)
   c. If none, look for word boundary (space)
   d. If exceeds MAX_SEGMENT_LENGTH, force break
3. Send segment to TTS
4. Wait for backpressure signal if needed
5. Repeat
```

## Backpressure Flow

```
Text arrives → Check control_signal → 
If "pause": Hold segments → 
If "resume": Send segments →
TTS processes → tts_complete signal →
Ready for next segment
```

## Usage Example

```yaml
nodes:
  - id: text-segmenter
    path: dora-text-segmenter
    inputs:
      text: qwen3-llm/text
      control_signal: conversation-controller/segment_control
      control: external/control
      tts_complete: primespeech/segment_complete
    outputs:
      - text_segment
      - status
    env:
      MAX_SEGMENT_LENGTH: 50
      MIN_SEGMENT_LENGTH: 10
      USE_BACKPRESSURE_CONTROL: true
```

## Segmentation Examples

### Input Text:
```
"今天天气很好，适合出去散步。明天可能会下雨，记得带伞。"
```

### Output Segments:
```
Segment 1: "今天天气很好，适合出去散步。"
Segment 2: "明天可能会下雨，记得带伞。"
```

### Long Text Handling:
```
Input: "This is a very long sentence that exceeds the maximum segment length and needs to be broken down into smaller chunks for processing"

Segment 1: "This is a very long sentence that exceeds the"
Segment 2: "maximum segment length and needs to be broken"
Segment 3: "down into smaller chunks for processing"
```

## Control Signals

### From conversation-controller:
- **`pause`**: Stop sending segments (buffer full)
- **`resume`**: Resume sending segments (buffer ready)

### From external:
- **`reset`**: Clear buffer and reset state
- **`flush`**: Force send remaining buffer content
- **`status`**: Get current state and statistics

## State Management

```
IDLE → BUFFERING → SEGMENTING → SENDING → WAITING
  ↑                                            ↓
  ←────────────────────────────────────────────
```

1. **IDLE**: No text to process
2. **BUFFERING**: Accumulating text
3. **SEGMENTING**: Finding break points
4. **SENDING**: Outputting segment
5. **WAITING**: Paused for backpressure

## Best Practices

1. **Segment Length**: 
   - Shorter (30-50 chars) for responsive feel
   - Longer (50-100 chars) for natural flow

2. **Backpressure**: 
   - Enable for smooth playback
   - Disable for testing/debugging

3. **Punctuation**: 
   - Ensure LLM outputs proper punctuation
   - Consider adding punctuation restoration

4. **Buffer Management**:
   - Monitor status output for buffer stats
   - Adjust thresholds based on TTS speed

## Statistics Output

```json
{
    "state": "sending",
    "buffer_size": 125,
    "segments_sent": 8,
    "segments_pending": 2,
    "total_chars_processed": 450,
    "backpressure_pauses": 3,
    "timestamp": 1234567890.123
}
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Choppy speech | Increase MIN_SEGMENT_LENGTH |
| Long delays | Decrease MAX_SEGMENT_LENGTH |
| Buffer overflow | Enable USE_BACKPRESSURE_CONTROL |
| Unnatural breaks | Ensure proper punctuation in LLM output |
| Segments too frequent | Increase MIN_SEGMENT_LENGTH |

## Algorithm Details

### Boundary Detection Priority:
1. **Sentence endings**: `. ! ? 。！？`
2. **Major pauses**: `; ；`
3. **Minor pauses**: `, ，、`
4. **Connectors**: `： :`
5. **Word boundaries**: spaces
6. **Forced break**: At MAX_SEGMENT_LENGTH

### Chinese Language Support:
- Detects Chinese punctuation
- No space-based word boundaries
- Character-based segmentation fallback

## Dependencies

- `pyarrow`: Dora communication
- `spacy`: NLP for sentence segmentation (optional)
- `re`: Regular expression processing