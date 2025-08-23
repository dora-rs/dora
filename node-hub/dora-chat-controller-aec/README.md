# Dora Chat Controller AEC

Simplified chat controller for voice chatbot with Acoustic Echo Cancellation (AEC).

This version doesn't need pause/resume logic since AEC handles echo cancellation automatically, enabling full-duplex conversation.

## Features

- No pause/resume state machine needed
- Manages conversation flow with question detection
- Maintains conversation history for context
- Simplified architecture for full-duplex conversation

## Installation

```bash
pip install -e .
```

## Usage

This node is designed to work with the voice-chatbot-aec example where AEC handles echo cancellation at the audio input level.