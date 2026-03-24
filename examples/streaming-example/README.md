# Streaming Pattern Example

Demonstrates token-by-token text generation using adora's streaming
pattern (session/segment/chunk metadata) with interruption support.

## Architecture

```
Prompt Source (0.5 Hz) --> Generator --> tokens (streamed) --> Sink
                           |                                   |
                           +-- session_id, seq, fin metadata --+
```

## Nodes

**prompt-source** (`prompt_source.py`) — Sends text prompts every 2 seconds.

**generator** (`generator.py`) — Simulates LLM token generation:
- Splits response into individual tokens
- Sends each token with streaming metadata:
  - `session_id`: unique per prompt (groups tokens into a response)
  - `segment_id`: "0" (single segment per response)
  - `seq`: 0, 1, 2, ... (sequence number within segment)
  - `fin`: "true" on last token (signals response complete)
- 50ms delay between tokens (simulates generation time)

**sink** (`sink.py`) — Reassembles token streams:
- Accumulates tokens by `session_id`
- Displays complete response when `fin=true`
- Warns about incomplete sessions on shutdown

## Run

```bash
pip install adora-rs pyarrow
adora run examples/streaming-example/dataflow.yml --uv --stop-after 15s
```

## Expected Output

```
[prompt-source] Sent prompt: Tell me about robotics
[generator]     Generating response for: Tell me about robotics
[generator]     Streamed 21 tokens (session abc12345)
[sink]          [session abc12345] Complete (21 tokens): Robotics combines mechanical...

[prompt-source] Sent prompt: Explain zero-copy IPC
[generator]     Generating response for: Explain zero-copy IPC
[generator]     Streamed 20 tokens (session def67890)
[sink]          [session def67890] Complete (20 tokens): Zero-copy IPC avoids...
```

## What This Demonstrates

| Feature | How It's Used |
|---------|--------------|
| Streaming metadata | `session_id`, `segment_id`, `seq`, `fin` keys |
| Token-by-token delivery | Each word sent as a separate message |
| Session reassembly | Sink accumulates by `session_id` until `fin` |
| Interruption support | `fin=true` signals clean completion |
| LLM integration pattern | Replace simulated tokens with real inference |

## Streaming Pattern Reference

See `docs/patterns.md` for the full streaming pattern specification:
- Session: groups related segments (e.g., one LLM response)
- Segment: subdivides a session (e.g., different audio channels)
- Chunk: individual data unit with sequence number
- Flush: interrupt signal to abort current segment
