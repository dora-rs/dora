# Chat History Configuration Guide

## Overview
The Qwen3 node now supports configurable chat history management with three strategies to optimize context usage.

## Configuration Options

### Environment Variables

```bash
# Basic history settings
export MAX_HISTORY_EXCHANGES=10      # Number of Q&A pairs to keep (default: 10)
export HISTORY_STRATEGY="fixed"      # Strategy: fixed, token_based, or sliding_window
export MAX_HISTORY_TOKENS=2000       # Max tokens for history (when using token_based)
```

### Strategies Explained

#### 1. Fixed Strategy (Default)
- Keeps a fixed number of recent exchanges
- Simple and predictable
- Best for: General conversations

```bash
export HISTORY_STRATEGY="fixed"
export MAX_HISTORY_EXCHANGES=10
```

#### 2. Token-Based Strategy
- Dynamically adjusts history based on estimated token count
- Maximizes context usage without overflow
- Best for: Variable-length conversations

```bash
export HISTORY_STRATEGY="token_based"
export MAX_HISTORY_TOKENS=2000
```

#### 3. Sliding Window (Future)
- Currently falls back to fixed strategy
- Planned: Summarize old conversations while keeping recent ones detailed

## Recommended Settings by Use Case

### Quick Q&A / Voice Assistant
```bash
export MAX_HISTORY_EXCHANGES=5
export HISTORY_STRATEGY="fixed"
```

### Technical Support / Debugging
```bash
export MAX_HISTORY_EXCHANGES=12
export HISTORY_STRATEGY="token_based"
export MAX_HISTORY_TOKENS=2500
```

### Creative Writing / Story Telling
```bash
export MAX_HISTORY_EXCHANGES=15
export HISTORY_STRATEGY="token_based"
export MAX_HISTORY_TOKENS=3000
```

### Casual Conversation
```bash
export MAX_HISTORY_EXCHANGES=8
export HISTORY_STRATEGY="fixed"
```

## Token Estimation Formula

For planning purposes:
- System prompt: ~50-100 tokens
- Average Q&A exchange: ~200-300 tokens
- Safety buffer: ~500 tokens

```
Available for history = CONTEXT_SIZE - SYSTEM_TOKENS - MAX_RESPONSE_TOKENS - BUFFER
Max exchanges = Available / AVG_TOKENS_PER_EXCHANGE
```

Example for 4096 context:
- Available: 4096 - 100 - 1000 - 500 = 2496 tokens
- Max exchanges: 2496 / 250 ≈ 10 exchanges

## Usage Example

```bash
# Start with optimized settings for voice chat
export HISTORY_STRATEGY="token_based"
export MAX_HISTORY_TOKENS=2000
export MAX_HISTORY_EXCHANGES=8

# Run the voice chatbot
cd examples/voice-chatbot-aec
./run_chatbot_aec.sh
```

## Monitoring History Usage

The node logs history management details when LOG_LEVEL=DEBUG:
```bash
export LOG_LEVEL=DEBUG
```

This will show:
- Number of messages kept/trimmed
- Estimated token usage
- Strategy decisions