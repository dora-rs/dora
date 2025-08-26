# Dora Text Segmenter Node

A text segmentation node for streaming LLM output to TTS systems with backpressure control.

## Overview

This node buffers streaming text from LLMs and sends it to TTS systems one segment at a time, preventing overwhelming the TTS with too much text and ensuring smooth, sequential speech synthesis.

## Default Implementation

The node now uses **queue-based segmenter** as the default implementation, which provides:
- No deadlock - first segment sent immediately
- Queue-based buffering of incoming text
- Backpressure control via TTS completion signals
- Automatic filtering of punctuation-only segments

## Usage

### As a Dora Node

```yaml
nodes:
  - id: text-segmenter
    build: pip install -e path/to/dora-text-segmenter
    path: dora-text-segmenter  # Uses queue_based_segmenter by default
    inputs:
      text: llm/text  # Streaming text from LLM
      tts_complete: tts/segment_complete  # Completion signal from TTS
    outputs:
      - text_segment  # Segmented text to TTS
      - status
      - metrics
```

### Running Standalone

```bash
# Run as module (uses queue_based_segmenter)
python -m dora_text_segmenter

# Run directly
python dora_text_segmenter/main.py
```

### Alternative Implementations

```bash
# Use specific segmenter
python dora_text_segmenter/queue_based_segmenter.py  # Queue-based (default)
python dora_text_segmenter/simple_passthrough.py     # No segmentation
python dora_text_segmenter/main_sequential.py        # Legacy sequential
```

## Configuration

Environment variables:
- `ENABLE_BACKPRESSURE`: Enable/disable backpressure control (default: true)
- `SEGMENT_MODE`: Segmentation mode (sentence/punctuation/fixed)
- `MIN_SEGMENT_LENGTH`: Minimum characters per segment (default: 5)
- `MAX_SEGMENT_LENGTH`: Maximum characters per segment (default: 100)
- `PUNCTUATION_MARKS`: Punctuation marks for segmentation (default: "。！？.!?")

## Key Features

### Queue-Based Segmenter (Default)
- **No deadlock**: First segment sent immediately without waiting
- **Smart buffering**: Queues segments while TTS is busy
- **Punctuation filtering**: Skips segments with only punctuation/numbers
- **Progressive processing**: Doesn't wait for complete sentences

### Backpressure Control
- Waits for TTS completion signal before sending next segment
- Prevents TTS buffer overflow
- Ensures smooth, sequential speech

## Input/Output

### Inputs
- `text`: Streaming text chunks from LLM
- `tts_complete`: Signal from TTS when segment is complete

### Outputs
- `text_segment`: Segmented text ready for TTS
- `status`: Node status messages
- `metrics`: Performance metrics

## Example Flow

```
LLM → "今天天气" → Segmenter (queues)
LLM → "很好。" → Segmenter (queues)
Segmenter → "今天天气很好。" → TTS
TTS → [processes] → segment_complete signal
Segmenter → [next segment] → TTS
```

## Development

To modify the default segmenter:
1. Edit `dora_text_segmenter/main.py`
2. Import your preferred implementation
3. Update the import statement to use your segmenter

## License

Part of the Dora ecosystem.