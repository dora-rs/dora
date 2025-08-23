# Conversation Controller Node

Backpressure control system for TTS-to-audio pipeline. Monitors audio buffer status and controls text segmentation flow to prevent overflow and ensure smooth playback.

## Overview

The Conversation Controller implements a feedback loop between the audio player's buffer and the text segmenter. It monitors buffer fullness and sends pause/resume signals to prevent buffer overflow while maintaining continuous audio playback.

The Conversation Controller monitors audio buffer fullness and controls text segment generation to prevent buffer overrun while maintaining smooth audio playback.

## Features

- **Hysteresis-based control**: Prevents oscillation with separate pause/resume thresholds
- **Configurable thresholds**: Adjust control points via environment variables
- **Session support**: Can manage multiple concurrent sessions
- **Statistics tracking**: Monitor control decisions and performance

## Installation

```bash
pip install -e .
```

## Usage

### In a Dora dataflow:

```yaml
nodes:
  - id: conversation-controller
    build: pip install -e ../../node-hub/dora-conversation-controller
    path: dora-conversation-controller
    inputs:
      buffer_status: audio-player/buffer_status
    outputs:
      - segment_control  # Control signals for text segmenter
      - status          # Optional status output
    env:
      PAUSE_THRESHOLD: "70"   # Pause when buffer > 70%
      RESUME_THRESHOLD: "40"  # Resume when buffer < 40%
      MIN_CONTROL_INTERVAL: "1.0"  # Minimum seconds between control changes
      STATUS_INTERVAL: "5.0"  # Status display interval
      LOG_LEVEL: "INFO"      # DEBUG, INFO, WARNING, ERROR
```

## Inputs

### buffer_status
- **Type**: Float (0-100)
- **Description**: Current buffer fullness percentage
- **Metadata**: Optional session_id, timestamp, etc.

## Outputs

### segment_control
- **Values**: "pause" | "resume"
- **Description**: Control signal for text segment generation
- **Metadata**:
  - `control`: The control signal
  - `buffer_percentage`: Current buffer level
  - `session_id`: Session identifier
  - `reason`: Reason for control change
  - `thresholds`: Current threshold configuration

### status (optional)
- **Type**: JSON string
- **Description**: Controller status and statistics
- **Sent**: Periodically based on STATUS_INTERVAL

## Configuration

| Variable | Default | Description |
|----------|---------|-------------|
| PAUSE_THRESHOLD | 70 | Buffer percentage to pause generation |
| RESUME_THRESHOLD | 40 | Buffer percentage to resume generation |
| MIN_CONTROL_INTERVAL | 1.0 | Minimum seconds between control changes |
| STATUS_INTERVAL | 5.0 | Seconds between status outputs |
| LOG_LEVEL | INFO | Logging verbosity |

## Control Logic

```
Buffer > PAUSE_THRESHOLD (70%):
  → Send "pause" signal
  → Stop text generation
  
Buffer < RESUME_THRESHOLD (40%):
  → Send "resume" signal
  → Resume text generation
```

The hysteresis gap (70% - 40% = 30%) prevents rapid oscillation.

## Example Integration

```
Text Input → Segmenter → TTS → Audio Player
               ↑                      ↓
         Controller ← buffer_status ──┘
```

The controller creates a feedback loop that automatically manages flow based on buffer status.