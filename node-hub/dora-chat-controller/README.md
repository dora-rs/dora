# Chat Controller Node

Central orchestrator for voice conversation systems. Manages conversation flow, accumulates transcriptions into questions, controls microphone pause/resume, and maintains conversation history.

## Overview

The Chat Controller is the brain of the voice chatbot system. It coordinates between speech detection, ASR transcription, LLM processing, and audio playback to create a seamless conversational experience with automatic turn-taking.

## Features

- **Transcription Accumulation**: Builds complete questions from ASR segments
- **Conversation State Management**: Tracks listening, processing, and answering states
- **Automatic Microphone Control**: Pauses during AI responses to prevent feedback
- **Conversation History**: Maintains context for multi-turn conversations
- **Question Validation**: Filters out false triggers and noise
- **Buffer Monitoring**: Resumes listening when AI finishes speaking

## Installation

```bash
pip install -e dora-chat-controller
```

## Inputs

| Input | Type | Description |
|-------|------|-------------|
| `speech_started` | Signal | User began speaking (from speech-monitor) |
| `speech_ended` | Signal | Speech segment ended (from speech-monitor) |
| `question_ended` | Signal | Complete question detected (from speech-monitor) |
| `transcription` | String | ASR transcription segments (from ASR) |
| `buffer_status` | Dict | Audio playback buffer status (from audio-player) |
| `control` | String | External control commands |

## Outputs

| Output | Type | Description |
|--------|------|-------------|
| `question_to_llm` | String | Complete questions sent to LLM |
| `speech_control` | String | Pause/resume commands for speech-monitor |
| `transcription_display` | String | Real-time transcription for UI |
| `status` | Dict | Conversation state and statistics |
| `log` | String | Debug and status messages |

## Configuration

```yaml
env:
  # Question validation
  MIN_QUESTION_LENGTH: 5              # Minimum characters for valid question
  
  # Conversation memory
  MAX_CONVERSATION_HISTORY: 10        # Number of Q&A pairs to remember
  
  # Answer completion detection
  ANSWER_COMPLETE_BUFFER_THRESHOLD: 1.0  # Buffer % to consider answer done
  
  # Logging
  LOG_LEVEL: INFO                     # DEBUG, INFO, WARNING, ERROR
```

## State Machine

```
LISTENING → PROCESSING → ANSWERING → LISTENING
    ↑                                    ↓
    ←────────────────────────────────────
```

1. **LISTENING**: Accumulating user transcriptions
2. **PROCESSING**: Question sent to LLM, microphone paused
3. **ANSWERING**: AI response playing, waiting for completion

## Conversation Flow

```
1. User speaks
2. Controller accumulates transcriptions
3. 3-second silence detected (question_ended)
4. Controller validates question (>5 chars)
5. Sends question to LLM
6. Pauses microphone (prevent feedback)
7. LLM processes and responds
8. TTS generates audio
9. Audio plays through speaker
10. Buffer empties (<1%)
11. Controller resumes microphone
12. Ready for next question
```

## Usage Example

```yaml
nodes:
  - id: chat-controller
    path: dora-chat-controller
    inputs:
      speech_started: speech-monitor/speech_started
      speech_ended: speech-monitor/speech_ended
      question_ended: speech-monitor/question_ended
      transcription: asr/transcription
      buffer_status: audio-player/buffer_status
      control: external/control
    outputs:
      - question_to_llm
      - speech_control
      - transcription_display
      - status
      - log
    env:
      MIN_QUESTION_LENGTH: 5
      MAX_CONVERSATION_HISTORY: 10
      ANSWER_COMPLETE_BUFFER_THRESHOLD: 1.0
```

## Control Commands

- **`reset`**: Clear conversation history and reset state
- **`status`**: Get current state and statistics
- **`history`**: Retrieve conversation history

## Event Handlers

### on_speech_started
- Resets accumulated transcriptions
- Updates state to LISTENING

### on_transcription
- Accumulates new transcription segment
- Sends to display output for UI
- Ignores if in ANSWERING state

### on_question_ended
- Validates accumulated question length
- Sends complete question to LLM
- Pauses speech monitor
- Updates conversation history
- Transitions to PROCESSING state

### on_buffer_status
- Monitors audio playback progress
- When buffer < 1%:
  - Resumes speech monitor
  - Transitions to LISTENING state

## Conversation History

Maintains a rolling window of Q&A pairs:

```python
{
    "history": [
        {"question": "What's the weather?", "answer": "It's sunny today"},
        {"question": "How about tomorrow?", "answer": "Rain expected"}
    ],
    "count": 2
}
```

## Best Practices

1. **Tune MIN_QUESTION_LENGTH** to filter noise while allowing short queries
2. **Adjust ANSWER_COMPLETE_BUFFER_THRESHOLD** based on TTS latency
3. **Set appropriate CONVERSATION_HISTORY** for context needs
4. **Monitor status output** for conversation metrics
5. **Handle control commands** for production reset capabilities

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Microphone doesn't resume | Check buffer_status is being sent, adjust threshold |
| False questions sent | Increase MIN_QUESTION_LENGTH |
| Context lost | Increase MAX_CONVERSATION_HISTORY |
| Feedback/echo heard | Ensure pause command reaches speech-monitor |
| Transcriptions cut off | Check question_ended timing with speech-monitor |

## Statistics Output

The status output provides real-time metrics:

```json
{
    "state": "listening",
    "questions_processed": 5,
    "current_transcription": "What is",
    "history_size": 5,
    "last_question": "What is the time?",
    "timestamp": 1234567890.123
}
```

## Dependencies

- `pyarrow`: Dora communication
- `numpy`: Data processing
- `time`: Timing and timestamps