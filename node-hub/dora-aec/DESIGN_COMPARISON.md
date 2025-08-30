# AEC Node Design Comparison

## Current Design (Multi-threaded with Queues)

### Architecture
```
┌─────────────────────────────────────┐
│         AEC Node (Single)           │
│  ┌──────────────────────────────┐   │
│  │   Main Thread (Dora Loop)    │   │
│  │  - Handle events              │   │
│  │  - Send outputs from queue    │   │
│  └──────────────────────────────┘   │
│  ┌──────────────────────────────┐   │
│  │  Processing Thread            │   │
│  │  - Poll native library        │   │
│  │  - Convert audio format       │   │
│  │  - Queue outputs              │   │
│  └──────────────────────────────┘   │
│         ↓ Queues ↓                  │
│  [Audio Queue] [VAD Queue] [Logs]   │
└─────────────────────────────────────┘
```

### Pros
- Single node deployment
- Internal state management
- Thread isolation for blocking calls

### Cons
- Complex internal logic
- Hard to debug threading issues
- Mixes multiple responsibilities
- Not leveraging Dora's strengths

## Redesigned Option 1: Simple Event-Driven

### Architecture
```
┌─────────────────────────────────────┐
│    Simple AEC Node (No Threading)   │
│                                     │
│  while True:                        │
│    audio = poll_native_lib()  # Non-blocking
│    if audio:                        │
│      send_output(audio)             │
│    event = node.next(0.001)   # 1ms │
│    handle_event(event)              │
└─────────────────────────────────────┘
```

### Implementation
- Single event loop
- Non-blocking polling
- Direct output sending
- ~50 lines of code

### Pros
- **Simplicity** - No threading complexity
- **Maintainable** - Easy to understand and debug
- **Efficient** - Low overhead
- **Reliable** - No synchronization issues

### Cons
- Must ensure native calls don't block
- Polling interval affects latency

## Redesigned Option 2: Multi-Node Pipeline

### Architecture
```
┌──────────┐    ┌──────────┐    ┌──────────┐
│   AEC    │───>│   Audio  │───>│  Audio   │
│ Capture  │    │Normalizer│    │  Buffer  │
└──────────┘    └──────────┘    └──────────┘
                                      │
┌──────────┐    ┌──────────┐         ↓
│Microphone│───>│  Audio   │    ┌──────────┐
│ Fallback │    │ Selector │───>│  Speech  │
└──────────┘    └──────────┘    │ Monitor  │
                                 └──────────┘
```

### Node Responsibilities
1. **aec-capture**: Hardware audio capture only
2. **audio-normalizer**: Format conversion (int16→float32)
3. **audio-buffer**: Chunk accumulation
4. **audio-selector**: Source switching
5. **microphone-fallback**: Backup audio source

### Pros
- **Single Responsibility** - Each node has one job
- **Composable** - Mix and match nodes
- **Parallel Processing** - True pipeline parallelism
- **Debuggable** - Can tap into any stream
- **Scalable** - Add nodes without changing others
- **Reusable** - Nodes work in other pipelines

### Cons
- More nodes to deploy
- Inter-node communication overhead
- Configuration complexity

## Recommendation

### For Production: **Option 1 (Simple Event-Driven)**
```python
# Complete implementation in ~50 lines
def main():
    node = Node()
    aec = SimpleAECNode()
    
    while True:
        # Non-blocking poll
        audio = aec.poll_audio()
        if audio:
            node.send_output("audio", audio)
        
        # Handle events
        event = node.next(timeout=0.001)
        if event:
            handle_event(event)
```

**Why:**
- Immediate improvement over current design
- Drop-in replacement
- Eliminates threading complexity
- Easy to maintain

### For Advanced Use Cases: **Option 2 (Pipeline)**
Use when you need:
- Different audio sources
- Complex processing chains
- Debugging/monitoring capabilities
- Maximum flexibility

## Migration Path

1. **Phase 1**: Replace current node with Simple Event-Driven
2. **Phase 2**: Test and validate in production
3. **Phase 3**: Consider pipeline for advanced features

## Key Insight

The native library already handles the hard part (AEC). The node should just:
1. Poll for audio (non-blocking)
2. Convert format
3. Send to next node

No need for complex orchestration - let Dora handle that!