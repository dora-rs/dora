# TODO - OpenAI Realtime API Implementation

## Overview
Current implementation covers ~40% of the OpenAI Realtime API specification. This document tracks remaining work needed for full API compliance.

## 🔴 Critical Features (High Priority)

### 1. Function Calling / Tool Support
**Status**: 0% implemented  
**Required for**: AI assistants that can take actions

- [ ] Add `tools` array to session configuration
- [ ] Implement `tool_choice` parameter
- [ ] Add function call events:
  - [ ] `response.function_call_arguments.delta`
  - [ ] `response.function_call_arguments.done`
- [ ] Support function call conversation items:
  - [ ] `function_call` item type
  - [ ] `function_call_output` item type
- [ ] Create function execution framework
- [ ] Add function result handling

**Implementation sketch**:
```rust
pub struct FunctionCall {
    name: String,
    arguments: String,
    call_id: String,
}

pub struct FunctionDefinition {
    name: String,
    description: String,
    parameters: serde_json::Value,
}
```

### 2. Response Lifecycle Management
**Status**: 30% implemented  
**Required for**: Proper response tracking and interruption handling

- [ ] Add response state tracking
- [ ] Implement missing events:
  - [ ] `response.created` - Response started
  - [ ] `response.done` - Response completed
  - [ ] `response.cancelled` - Response cancelled
- [ ] Add response ID generation and tracking
- [ ] Implement `response.cancel` client event
- [ ] Handle response interruption properly
- [ ] Track output indices for multi-part responses

**Implementation sketch**:
```rust
pub struct ResponseState {
    id: String,
    status: ResponseStatus,
    created_at: u64,
    items: Vec<String>,
}

enum ResponseStatus {
    Created,
    InProgress,
    Done,
    Cancelled,
}
```

### 3. Voice Activity Detection (VAD) Configuration
**Status**: Hardcoded implementation  
**Required for**: Automatic turn-taking in conversations

- [ ] Make VAD configurable via session.update
- [ ] Implement proper VAD parameters:
  - [ ] `threshold` (0.0-1.0)
  - [ ] `silence_duration_ms`
  - [ ] `prefix_padding_ms`
- [ ] Add VAD-triggered response creation
- [ ] Implement interrupt detection
- [ ] Add VAD enable/disable toggle

**Current hardcoded values to make configurable**:
```rust
// Currently in speech-monitor, needs to be configurable
MIN_AUDIO_AMPLITUDE: 0.01
ACTIVE_FRAME_THRESHOLD_MS: 100
USER_SILENCE_THRESHOLD_MS: 1500
VAD_THRESHOLD: 0.6
```

## 🟡 Important Features (Medium Priority)

### 4. Conversation State Management
**Status**: Minimal implementation  
**Required for**: Context management and conversation history

- [ ] Add conversation tracking:
  - [ ] `conversation.created` event
  - [ ] Conversation ID management
  - [ ] Conversation history storage
- [ ] Implement item management:
  - [ ] `conversation.item.delete` event
  - [ ] `conversation.item.deleted` confirmation
  - [ ] Item status tracking
- [ ] Add conversation context limits
- [ ] Implement conversation truncation

### 5. Input Audio Transcription
**Status**: Not implemented  
**Required for**: Debugging, logging, and accessibility

- [ ] Add Whisper transcription configuration
- [ ] Implement transcription events:
  - [ ] `conversation.item.input_audio_transcription.completed`
  - [ ] `conversation.item.input_audio_transcription.failed`
- [ ] Add transcript storage
- [ ] Support transcript display in responses
- [ ] Add language detection from transcripts

### 6. Rate Limiting & Usage Tracking
**Status**: Not implemented  
**Required for**: API compliance and cost management

- [ ] Add rate limit tracking:
  - [ ] `rate_limits.updated` event
  - [ ] Token usage monitoring
  - [ ] Request count tracking
- [ ] Implement rate limit headers parsing
- [ ] Add usage metrics:
  - [ ] Text tokens (input/output)
  - [ ] Audio tokens (input/output)
  - [ ] Total cost calculation
- [ ] Add rate limit retry logic

### 7. Enhanced Error Handling
**Status**: Basic implementation  
**Required for**: Production stability

- [ ] Add comprehensive error types:
  - [ ] Connection errors
  - [ ] Authentication errors
  - [ ] Rate limit errors
  - [ ] Invalid request errors
- [ ] Implement automatic reconnection
- [ ] Add exponential backoff
- [ ] Create error recovery strategies
- [ ] Add detailed error logging

## 🟢 Nice-to-Have Features (Low Priority)

### 8. Additional Audio Formats
**Status**: PCM16 24kHz only  
**Required for**: Bandwidth optimization

- [ ] Support additional PCM sample rates:
  - [ ] 8kHz
  - [ ] 16kHz
- [ ] Add G.711 codec support:
  - [ ] µ-law
  - [ ] A-law
- [ ] Implement format negotiation
- [ ] Add automatic format detection

### 9. Advanced Session Features
**Status**: Basic implementation  
**Required for**: Advanced use cases

- [ ] Add modality selection:
  - [ ] Text-only mode
  - [ ] Audio-only mode
  - [ ] Mixed modalities
- [ ] Implement `max_response_output_tokens`
- [ ] Add custom instructions per response
- [ ] Support temperature adjustment
- [ ] Add streaming configuration options

### 10. Client Event Completeness
**Status**: Core events only  
**Required for**: Full API compatibility

- [ ] Add missing client events:
  - [ ] `input_audio_buffer.clear`
  - [ ] `response.cancel`
- [ ] Add missing server confirmations:
  - [ ] `input_audio_buffer.committed`
  - [ ] `input_audio_buffer.cleared`
- [ ] Implement event acknowledgments
- [ ] Add event sequence validation

## 📊 Implementation Progress

| Component | Implementation | Priority | Effort |
|-----------|---------------|----------|--------|
| Function Calling | 0% | High | Large |
| Response Lifecycle | 30% | High | Medium |
| VAD Configuration | 20% | High | Medium |
| Conversation State | 10% | Medium | Medium |
| Input Transcription | 0% | Medium | Medium |
| Rate Limiting | 0% | Medium | Small |
| Error Handling | 20% | Medium | Medium |
| Audio Formats | 20% | Low | Small |
| Advanced Session | 30% | Low | Small |
| Event Completeness | 40% | Low | Medium |

## 🚀 Quick Wins

These items can be implemented quickly for immediate improvement:

1. **Add response.created event** - Simple event addition
2. **Make VAD configurable** - Move hardcoded values to config
3. **Add input_audio_buffer.clear** - Simple buffer operation
4. **Add basic rate limit tracking** - Log token usage
5. **Add response IDs** - Generate UUIDs for responses

## 🏗️ Architecture Recommendations

### Event System Refactor
Consider refactoring to a more complete event system:
```rust
// Centralized event handler
pub struct EventHandler {
    session: SessionState,
    conversation: ConversationState,
    responses: HashMap<String, ResponseState>,
    rate_limits: RateLimits,
}

// Event processing pipeline
impl EventHandler {
    pub async fn process_client_event(&mut self, event: ClientEvent) -> Result<Vec<ServerEvent>>;
    pub async fn process_dora_event(&mut self, event: DoraEvent) -> Result<Vec<ServerEvent>>;
}
```

### State Management
Add proper state management:
```rust
pub struct RealtimeState {
    session_id: String,
    conversation_id: String,
    active_response: Option<String>,
    pending_functions: Vec<FunctionCall>,
    audio_buffer: AudioBuffer,
    transcript_buffer: String,
}
```

### Testing Infrastructure
Add comprehensive testing:
- [ ] Unit tests for each event type
- [ ] Integration tests for event sequences
- [ ] Load tests for audio streaming
- [ ] Compliance tests against API spec

## 📝 Notes

- Current focus has been on audio quality and resampling (working well)
- WebSocket infrastructure is solid
- Dora integration is functional
- Main gaps are in conversational features and API completeness
- Function calling is the most critical missing feature for AI assistant use cases

## 🎯 Recommended Implementation Order

1. **Phase 1** (Core Functionality):
   - Response lifecycle events
   - VAD configuration
   - Basic function calling

2. **Phase 2** (Production Ready):
   - Error handling improvements
   - Rate limiting
   - Input transcription

3. **Phase 3** (Full Compliance):
   - Conversation management
   - Additional audio formats
   - Complete event coverage

## 📚 References

- [OpenAI Realtime API Documentation](https://platform.openai.com/docs/guides/realtime)
- [API Reference](https://platform.openai.com/docs/api-reference/realtime)
- See [API_COMPARISON.md](./API_COMPARISON.md) for detailed gap analysis
- See [RESAMPLING_HISTORY.md](./RESAMPLING_HISTORY.md) for audio implementation details