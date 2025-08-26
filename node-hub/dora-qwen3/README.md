# Qwen3 LLM Node

High-performance Large Language Model node featuring Qwen3 with automatic hardware optimization. Supports both MLX (Apple Silicon) and GGUF (cross-platform) backends with streaming generation and KV caching.

## Overview

The Qwen3 node provides intelligent conversational AI capabilities optimized for real-time voice interactions. It automatically detects hardware and uses the best available backend - MLX for Apple Silicon or GGUF for other systems.

## Features

- **Automatic Hardware Detection**: Selects MLX on Apple Silicon, GGUF elsewhere
- **Multiple Model Sizes**: 1B to 70B parameters
- **Streaming Generation**: Token-by-token output for responsive TTS
- **KV Cache Optimization**: Faster multi-turn conversations with prompt caching
- **Progressive Text Segmentation**: Sends complete sentences as they're generated
- **Conversation Context**: Maintains chat history with efficient caching
- **Bilingual Support**: Optimized for Chinese and English
- **System Prompts**: Customizable assistant personality
- **Temperature Control**: Adjustable response creativity

## Installation

```bash
pip install -e .
```

Or directly from the directory:
```bash
cd dora-qwen3
pip install .
```

## Usage

### In a Dora Dataflow

```yaml
nodes:
  - id: qwen3-llm
    operator: dora-qwen3
    inputs:
      text: chat-client/text
      control: chat-client/control
    outputs:
      - response
      - status
    env:
      USE_MLX: auto  # auto, true, or false
      ENABLE_THINKING: false
      MAX_TOKENS: 2048
```

### Environment Variables

#### Model Selection
- `USE_MLX`: Control MLX usage (auto/true/false)
- `MLX_MODEL`: MLX model to use (default: Qwen/Qwen3-8B-MLX-4bit)
- `GGUF_MODEL`: GGUF model repository (default: Qwen/Qwen3-8B-GGUF)
- `GGUF_MODEL_FILE`: GGUF model file (default: Qwen3-8B-Q4_K_M.gguf)

#### Generation Settings
- `ENABLE_THINKING`: Enable thinking mode with <think> tags
- `MAX_TOKENS`: Maximum tokens to generate (default: 256 for voice)
- `TEMPERATURE`: Generation temperature (default: 0.7)
- `LLM_ENABLE_STREAMING`: Enable streaming generation (default: true)
- `SYSTEM_PROMPT`: Custom system prompt for the assistant

#### Performance Settings
- `N_GPU_LAYERS`: GPU layers for GGUF (0 for CPU only)
- `N_THREADS`: Number of CPU threads for GGUF
- `CONTEXT_SIZE`: Context window size (default: 4096)
- `MAX_CONVERSATION_HISTORY`: Number of Q&A pairs to remember (default: 10)

#### Logging
- `LOG_LEVEL`: Logging level (DEBUG/INFO/WARNING/ERROR)

## Models

### MLX (Apple Silicon)
- Model: `Qwen/Qwen3-8B-MLX-4bit`
- Performance: ~0.3s first token latency
- Memory: ~4.5GB
- Features: KV caching, streaming generation

### GGUF (CPU/GPU)
- Model: `Qwen/Qwen3-8B-GGUF`
- Performance: ~1.3s average response time
- Memory: ~5GB
- Features: CPU/GPU acceleration

## Advanced Features

### Streaming Generation

The node supports real-time streaming of generated text, sending complete sentences as they're produced:

```yaml
env:
  LLM_ENABLE_STREAMING: "true"  # Enable streaming (default)
```

**How it works:**
1. Generates tokens progressively using `stream_generate`
2. Accumulates tokens into complete sentences
3. Sends each complete sentence immediately to downstream nodes
4. Enables TTS to start speaking before full response is complete

**Benefits:**
- Reduced perceived latency (first audio ~1-2s instead of 3-5s)
- More natural conversation flow
- Better handling of long responses

### KV Cache Optimization

For multi-turn conversations, the node uses KV (Key-Value) caching to speed up generation:

```python
# Automatically enabled when conversation history exists
# Caches attention computations from previous prompts
# Reduces generation time by 30-50% for follow-up questions
```

**How it works:**
1. Creates prompt cache using `make_prompt_cache` from MLX
2. Reuses cached attention keys/values for conversation context
3. Only computes new tokens, not entire prompt
4. Automatically managed per session

**Benefits:**
- Faster response times in ongoing conversations
- Lower memory bandwidth usage
- Maintains full conversation context efficiently

### Progressive Text Segmentation

When streaming is enabled, the node intelligently segments text:

```python
# Sends text at natural boundaries:
- Complete sentences (。！？.!?)
- Natural pauses (，,；;)
- Minimum segment length before sending
```

This ensures downstream TTS nodes receive meaningful chunks for synthesis.

## Integration with Voice Chat

### Optimal Configuration for Voice

```yaml
nodes:
  - id: qwen3-llm
    path: dora-qwen3
    inputs:
      text: chat-controller/question_to_llm
    outputs:
      - text
      - status
      - log
    env:
      USE_MLX: "auto"
      LLM_ENABLE_STREAMING: "true"
      MAX_TOKENS: "256"  # Concise for voice
      TEMPERATURE: "0.7"  # Natural conversation
      SYSTEM_PROMPT: "你的回答第一句话必须少于十个字。每段回答控制在二到三句话。"
```

### Data Flow

1. **Input**: Receives transcribed text from ASR
2. **Processing**: 
   - Uses KV cache for context
   - Streams tokens as generated
   - Segments at sentence boundaries
3. **Output**: 
   - Sends complete sentences progressively
   - Metadata includes streaming status and segment index
   - Compatible with text-segmenter and TTS nodes

## Performance Metrics

### With Streaming + KV Cache (MLX)
- First token: ~300ms
- First sentence: ~800ms
- Complete response: ~2-3s
- Memory: ~4.5GB constant

### Without Optimizations
- Full response: ~3-5s
- Memory: ~5-6GB variable

## Troubleshooting

### Streaming not working
- Check `LLM_ENABLE_STREAMING=true`
- Verify MLX is enabled (streaming only works with MLX)
- Check downstream nodes can handle streaming input

### Slow responses
- Enable KV cache (automatic with conversation history)
- Reduce MAX_TOKENS for voice applications
- Use quantized models (4-bit recommended)

### Memory issues
- Use smaller models (Qwen3-1B for low memory)
- Enable quantization (4-bit or 8-bit)
- Limit conversation history

## License

MIT