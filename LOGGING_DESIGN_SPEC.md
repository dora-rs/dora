# Dora Unified Logging System - Technical Design Specification

**Version:** 1.0
**Status:** Draft
**Authors:** Dora Team
**Last Updated:** 2025-01-08

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [Background & Motivation](#background--motivation)
3. [Architecture Overview](#architecture-overview)
4. [Component Specifications](#component-specifications)
5. [API Design](#api-design)
6. [Data Formats](#data-formats)
7. [Implementation Plan](#implementation-plan)
8. [Performance Considerations](#performance-considerations)
9. [Security & Privacy](#security--privacy)
10. [Testing Strategy](#testing-strategy)
11. [Migration Guide](#migration-guide)
12. [Alternatives Considered](#alternatives-considered)
13. [Open Questions](#open-questions)

---

## Executive Summary

This document specifies a unified logging system for dora-rs that addresses three key requirements:

1. **Cross-language consistency**: Unified APIs for Rust, Python, and C/C++ nodes
2. **Flexible routing**: Logs as data that can flow through the dataflow graph
3. **Enhanced observability**: Better filtering, aggregation, and external integrations

The design builds on existing infrastructure (`dora-tracing`, `LogMessage`, `send_stdout_as`) and introduces three new components:

- Cross-language logging APIs
- Enhanced `dora logs` CLI with real-time streaming
- Optional log aggregator node for unified timelines

---

## Background & Motivation

### Current Logging Architecture

```
┌──────────────┐
│ Rust Node    │ ──> tracing::info!() ──> dora-tracing ──> stdout
└──────────────┘

┌──────────────┐
│ Python Node  │ ──> print() ──────────────────────────> stdout
└──────────────┘
                                                            ↓
                                                    ┌──────────────┐
                                                    │ Daemon       │
                                                    │ - Captures   │
                                                    │ - Writes to  │
                                                    │   log file   │
                                                    └──────────────┘
                                                            ↓
                                                    log_<node>.txt
```

### Problems Identified

1. **Inconsistent APIs**: Rust uses `tracing`, Python uses `print()` or `logging`, C/C++ has no API
2. **No structured output**: Plain text logs are hard to parse and filter
3. **Limited routing**: Logs go to files, can't be processed in dataflow
4. **Poor correlation**: Hard to build unified timelines across nodes
5. **Hidden features**: `LogSubscribe` exists but isn't exposed to users

---

### How `dora logs` Currently Works

The `dora logs` command provides the foundation we'll build on. Understanding it is critical to the design.

#### Architecture

```
┌─────────┐         ┌─────────────┐         ┌────────┐         ┌──────────┐
│ dora    │ ─────>  │ coordinator │ ─────>  │ daemon │ ─────>  │ log file │
│ logs    │  TCP    │             │  TCP    │        │  read   │          │
└─────────┘         └─────────────┘         └────────┘         └──────────┘
```

#### Flow

1. **CLI** (`dora logs <dataflow> <node>`) sends `ControlRequest::Logs` to coordinator
2. **Coordinator** determines which daemon is running the specified node
3. **Coordinator** forwards `DaemonCoordinatorEvent::Logs` to that daemon
4. **Daemon** reads the log file from disk: `<working_dir>/out/<dataflow_id>/log_<node_id>.txt`
5. **Daemon** sends entire file contents back to coordinator
6. **Coordinator** forwards to CLI
7. **CLI** displays using `bat` (syntax-highlighted pager)

#### Key Implementation Details

**Log File Location:**
- Path: `<working_dir>/out/<dataflow_id>/log_<node_id>.txt`
- Code: `binaries/daemon/src/log.rs:24-27`

**Log Writing:**
- Daemon captures stdout/stderr of each node process
- Writes to file asynchronously (buffered)
- Code: `binaries/daemon/src/spawn.rs:585-608`

**Historical Access:**
- Reads entire file from disk when requested
- Works for both running and finished dataflows

**Hidden Features:**
- **`LogSubscribe` mechanism** (`ControlRequest::LogSubscribe`)
  - Enables real-time log streaming
  - Currently not exposed in CLI
  - Will be used for `--follow` enhancement

**Current Capabilities:**
- ✅ Per-node log files
- ✅ Historical log retrieval
- ✅ Works in distributed mode (coordinator routes to correct daemon)
- ✅ Preserves logs after dataflow finishes
- ❌ No filtering (gets entire file)
- ❌ No real-time streaming (only batch retrieval)
- ❌ No unified view across nodes
- ❌ No structured log parsing

---

### Design Goals

| Goal | Description | Priority |
|------|-------------|----------|
| **Consistency** | Same API feel across Rust/Python/C/C++ | P0 |
| **Flexibility** | Logs as data that can be routed | P0 |
| **Performance** | Minimal overhead on hot paths | P0 |
| **Discoverability** | Enhanced CLI with better UX | P1 |
| **Integration** | Easy export to Grafana, etc. | P2 |

---

## Architecture Overview

### High-Level Design

```
┌─────────────────────────────────────────────────────────────────┐
│                        User Code Layer                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐         │
│  │ Rust Node    │  │ Python Node  │  │ C/C++ Node   │         │
│  │              │  │              │  │              │         │
│  │ node.log()   │  │ node.log()   │  │ dora_log()   │         │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘         │
│         │                 │                 │                  │
└─────────┼─────────────────┼─────────────────┼──────────────────┘
          ↓                 ↓                 ↓
┌─────────────────────────────────────────────────────────────────┐
│                      Logging Facade                             │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌───────────────────────────────────────────────────────┐    │
│  │ DoraLogger (Rust)                                     │    │
│  │ - Converts to LogMessage                              │    │
│  │ - Adds context (node_id, timestamp, HLC)              │    │
│  │ - Routes to outputs                                   │    │
│  └───────────────────────────────────────────────────────┘    │
│                           │                                     │
└───────────────────────────┼─────────────────────────────────────┘
                            ↓
          ┌─────────────────┴──────────────────┐
          ↓                                    ↓
┌──────────────────────┐            ┌──────────────────────┐
│ Output Destination 1 │            │ Output Destination 2 │
├──────────────────────┤            ├──────────────────────┤
│ __dora_logs output   │            │ File (log_node.txt)  │
│ - Arrow format       │            │ - Plain text         │
│ - Shared memory      │            │ - Human readable     │
│ - To dataflow        │            │ - For dora logs cmd  │
└──────────────────────┘            └──────────────────────┘
          │
          ↓
┌──────────────────────┐
│ Log Aggregator Node  │
├──────────────────────┤
│ - Merges timelines   │
│ - Parses JSON        │
│ - Filters/routes     │
│ - Exports externally │
└──────────────────────┘
```

### Data Flow

**Path 1: Dataflow-native logging**
```
Node → DoraLogger → Arrow Array → Shared Memory → Log Aggregator → Outputs
```

**Path 2: File-based logging (current)**
```
Node → DoraLogger → Daemon Stdout Capture → File → dora logs CLI
```

**Path 3: Real-time streaming**
```
Node → DoraLogger → Daemon → Coordinator → LogSubscriber → CLI
```

All three paths coexist and serve different purposes.

### Integration with Existing `dora logs`

A critical design principle: **All approaches write to the same log files**, ensuring backward compatibility.

```
┌─────────────────────────────────────────────────────────┐
│                    User's Perspective                    │
├─────────────────────────────────────────────────────────┤
│ dora logs <dataflow> <node>     ← Historical logs       │
│ dora logs -f <dataflow> <node>  ← Real-time streaming   │
│ dora logs --all-nodes           ← Unified view          │
└─────────────────────────────────────────────────────────┘
                         ↓
         ┌───────────────┴───────────────┐
         ↓                               ↓
┌────────────────┐              ┌──────────────────┐
│  File-based    │              │ Dataflow-native  │
│  (Current)     │              │  (Enhanced)      │
├────────────────┤              ├──────────────────┤
│ • log files    │              │ • send_stdout_as │
│ • dora logs    │              │ • log aggregator │
│ • LogSubscribe │              │ • routing        │
└────────────────┘              └──────────────────┘
         ↓                               ↓
         └───────────────┬───────────────┘
                         ↓
              ┌──────────────────┐
              │  Same log files  │
              │  on disk!        │
              └──────────────────┘
```

#### Why This Works

1. **Node emits logs** (via `node.log()` or `print()`)
2. **Daemon captures stdout/stderr**
   - Always writes to `log_<node>.txt` file
   - If `send_stdout_as` configured → also sends to dataflow
3. **Both paths coexist:**
   - File path: For `dora logs` command, historical access
   - Dataflow path: For real-time processing, routing, aggregation
4. **Log aggregator node** (if present)
   - Receives logs from multiple nodes via dataflow
   - Outputs unified stream
   - Its output ALSO written to `log_log-aggregator.txt`

#### Example Usage

```bash
# Traditional: view individual node logs
dora logs my_dataflow asr

# Enhanced: real-time streaming with filtering
dora logs my_dataflow asr --follow --level error

# Dataflow-native: unified view from aggregator
dora logs my_dataflow log-aggregator --format json | jq '.node=="asr"'

# Multi-node: merged timeline
dora logs my_dataflow --all-nodes --since 1h
```

**Key Insight:** Users can choose their workflow without losing any capabilities. Traditional file-based logging still works, enhanced features are opt-in.

---

## Component Specifications

### 1. DoraLogger (Core Logging Facade)

**Location:** `apis/rust/node/src/logger.rs` (new file)

**Responsibilities:**
- Provide ergonomic logging API
- Convert to `LogMessage` struct
- Route to multiple destinations
- Add contextual metadata (node_id, HLC timestamp)

**Interface:**
```rust
pub struct DoraLogger {
    node_id: NodeId,
    clock: Arc<HLC>,
    log_output: Option<DataId>,  // For __dora_logs output
    file_logger: Option<FileLogger>,  // For traditional logs
}

impl DoraLogger {
    pub fn info(&self, message: impl Into<String>) -> LogBuilder;
    pub fn warn(&self, message: impl Into<String>) -> LogBuilder;
    pub fn error(&self, message: impl Into<String>) -> LogBuilder;
    pub fn debug(&self, message: impl Into<String>) -> LogBuilder;
    pub fn trace(&self, message: impl Into<String>) -> LogBuilder;
}

pub struct LogBuilder {
    message: String,
    level: LogLevel,
    fields: HashMap<String, serde_json::Value>,
}

impl LogBuilder {
    pub fn field(mut self, key: impl Into<String>, value: impl Serialize) -> Self;
    pub fn fields(mut self, fields: HashMap<String, serde_json::Value>) -> Self;
    pub fn send(self) -> Result<()>;  // Actually emit the log
}
```

**Usage Example:**
```rust
use dora_node_api::DoraNode;

let (node, events) = DoraNode::init_from_env()?;

// Simple logging
node.log().info("Processing started").send()?;

// Structured logging with fields
node.log()
    .info("Transcription complete")
    .field("question_id", 485057)
    .field("language", "zh")
    .field("duration_ms", 1200)
    .send()?;

// Error logging
node.log()
    .error("Failed to load model")
    .field("path", "/models/model.bin")
    .field("error", err.to_string())
    .send()?;
```

### 2. Python Logging API

**Location:** `apis/python/node/dora/log.py` (new file)

**Implementation via PyO3:**
```python
# apis/python/node/src/log.rs (Rust side)
#[pyclass]
pub struct DoraLogger {
    inner: Arc<Mutex<dora_node_api::DoraLogger>>,
}

#[pymethods]
impl DoraLogger {
    pub fn info(&self, message: &str, **kwargs) -> LogBuilder;
    pub fn error(&self, message: &str, **kwargs) -> LogBuilder;
    // ...
}
```

**Python API:**
```python
# dora/__init__.py
from .dora import Node, DoraLogger

node = Node()
log = node.logger()  # Get logger instance

# Usage
log.info("Processing started")
log.info("User speaking", question_id=485057, language="zh")
log.error("Model not found", path="/models/model.bin")

# Or chained
log.info("Processing").field("duration", 1.2).send()
```

**Alternative: Integrate with Python's logging module**
```python
import logging
from dora import configure_logging

# Configure Python's logging to use Dora backend
configure_logging(node)

logger = logging.getLogger(__name__)
logger.info("Message", extra={"question_id": 485057})
```

### 3. C/C++ Logging API

**Location:** `apis/c/node/src/log.h` (new file)

**C API:**
```c
// C API
typedef struct dora_logger_t dora_logger_t;

dora_logger_t* dora_node_logger(dora_node_t* node);

void dora_log_info(dora_logger_t* logger, const char* message);
void dora_log_error(dora_logger_t* logger, const char* message);

// With fields (using variadic args or JSON string)
void dora_log_info_json(
    dora_logger_t* logger,
    const char* message,
    const char* fields_json  // '{"question_id": 485057}'
);
```

**C++ API:**
```cpp
// C++ wrapper
class DoraLogger {
public:
    void info(const std::string& message);
    void error(const std::string& message);

    // Structured logging
    LogBuilder info(const std::string& message);
};

class LogBuilder {
public:
    template<typename T>
    LogBuilder& field(const std::string& key, const T& value);

    void send();
};

// Usage
node.logger().info("Processing started");

node.logger()
    .info("Transcription complete")
    .field("question_id", 485057)
    .field("language", "zh")
    .send();
```

### 4. Enhanced `dora logs` CLI

**Location:** `binaries/cli/src/command/logs.rs`

**New Arguments:**
```rust
#[derive(Debug, Args)]
pub struct LogsArgs {
    /// Identifier of the dataflow
    pub dataflow: Option<String>,

    /// Show logs for the given node (or --all-nodes)
    pub node: Option<String>,

    /// Show logs from all nodes
    #[clap(long, conflicts_with = "node")]
    pub all_nodes: bool,

    /// Follow log output (real-time streaming)
    #[clap(short, long)]
    pub follow: bool,

    /// Filter by log level
    #[clap(long, value_name = "LEVEL")]
    pub level: Option<LogLevel>,  // error, warn, info, debug, trace

    /// Output format
    #[clap(long, value_name = "FORMAT", default_value = "pretty")]
    pub format: LogFormat,  // pretty, json, compact

    /// Show logs since duration ago (e.g., "1h", "30m", "2d")
    #[clap(long, value_name = "DURATION")]
    pub since: Option<String>,

    /// Show logs until duration ago
    #[clap(long, value_name = "DURATION")]
    pub until: Option<String>,

    /// Filter by grep pattern
    #[clap(long, value_name = "PATTERN")]
    pub grep: Option<String>,

    /// Number of lines to show (tail)
    #[clap(short = 'n', long, value_name = "NUMBER")]
    pub tail: Option<usize>,
}
```

**Implementation:**

```rust
impl Executable for LogsArgs {
    fn execute(self) -> eyre::Result<()> {
        if self.follow {
            // Use LogSubscribe for real-time streaming
            self.stream_logs()
        } else {
            // Read from files
            self.retrieve_historical_logs()
        }
    }
}

impl LogsArgs {
    fn stream_logs(&self) -> eyre::Result<()> {
        let mut session = connect_to_coordinator(...)?;

        // Send LogSubscribe request
        let request = ControlRequest::LogSubscribe {
            dataflow_id: self.resolve_dataflow_id()?,
            level: self.level.unwrap_or(LogLevel::Info),
        };

        // Stream logs
        loop {
            let log_message: LogMessage = receive_log_from_coordinator(&mut session)?;

            // Apply filters
            if !self.matches_filters(&log_message) {
                continue;
            }

            // Format and print
            self.print_log(&log_message)?;
        }
    }

    fn retrieve_historical_logs(&self) -> eyre::Result<()> {
        if self.all_nodes {
            // Retrieve logs from all nodes
            let nodes = self.get_dataflow_nodes()?;
            let mut all_logs = Vec::new();

            for node in nodes {
                let logs = self.retrieve_node_logs(&node)?;
                all_logs.extend(logs);
            }

            // Sort by timestamp
            all_logs.sort_by_key(|log| log.timestamp);

            // Print unified timeline
            for log in all_logs {
                self.print_log(&log)?;
            }
        } else {
            // Single node logs (current behavior)
            let logs = self.retrieve_node_logs(&self.node.unwrap())?;
            for log in logs {
                self.print_log(&log)?;
            }
        }
    }

    fn matches_filters(&self, log: &LogMessage) -> bool {
        // Filter by level
        if let Some(level) = &self.level {
            if log.level < *level {
                return false;
            }
        }

        // Filter by time range
        if let Some(since) = &self.since {
            let since_time = parse_duration(since)?;
            if log.timestamp < since_time {
                return false;
            }
        }

        // Filter by grep
        if let Some(pattern) = &self.grep {
            if !log.message.contains(pattern) {
                return false;
            }
        }

        true
    }

    fn print_log(&self, log: &LogMessage) -> eyre::Result<()> {
        match self.format {
            LogFormat::Pretty => {
                println!("[{}][{:5}][{}] {}",
                    log.timestamp,
                    log.level,
                    log.node_id,
                    log.message
                );
            }
            LogFormat::Json => {
                println!("{}", serde_json::to_string(log)?);
            }
            LogFormat::Compact => {
                println!("{} {}: {}",
                    log.timestamp,
                    log.node_id,
                    log.message
                );
            }
        }
        Ok(())
    }
}
```

**Usage Examples:**
```bash
# Follow logs from single node
dora logs my_dataflow asr --follow

# View errors from all nodes
dora logs my_dataflow --all-nodes --level error

# JSON output for processing
dora logs my_dataflow --all-nodes --format json | jq '.question_id == 485057'

# Recent logs
dora logs my_dataflow asr --since 1h --tail 100

# Grep for specific pattern
dora logs my_dataflow asr --grep "transcription complete"
```

### 5. Log Aggregator Node

**Location:** `libraries/log-aggregator/src/lib.rs` (new crate)

**Purpose:**
- Collect logs from multiple nodes
- Parse structured logs (JSON)
- Merge into unified timeline
- Route to different outputs (unified, alerts, exports)

**Configuration:**
```yaml
nodes:
  # ... application nodes with send_stdout_as: logs

  - id: log_aggregator
    path: dora-log-aggregator
    inputs:
      asr_logs: asr/logs
      llm_logs: llm/logs
      tts_logs: tts/logs
    outputs:
      - unified      # All logs merged by timestamp
      - alerts       # ERROR/WARN only
      - metrics      # Aggregated counts
    env:
      LOG_FORMAT: json
      ALERT_WEBHOOK: https://hooks.slack.com/...
      EXPORT_LOKI: http://loki:3100
```

**Implementation:**
```rust
use dora_node_api::{DoraNode, Event};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

#[derive(Debug, Deserialize, Serialize)]
struct StructuredLog {
    timestamp: f64,
    level: String,
    node: String,
    message: String,
    #[serde(flatten)]
    fields: BTreeMap<String, serde_json::Value>,
}

fn main() -> eyre::Result<()> {
    let (node, events) = DoraNode::init_from_env()?;

    // Buffer for ordering logs by timestamp
    let mut log_buffer: BTreeMap<u64, StructuredLog> = BTreeMap::new();

    for event in events {
        match event {
            Event::Input { id, data, metadata } => {
                // Parse log from input
                let log_str = parse_arrow_string(data)?;

                // Try to parse as JSON, fallback to plain text
                let mut log: StructuredLog = if let Ok(parsed) = serde_json::from_str(&log_str) {
                    parsed
                } else {
                    // Plain text log - create structured entry
                    StructuredLog {
                        timestamp: metadata.timestamp.get_time().as_u64(),
                        level: "INFO".to_string(),
                        node: extract_node_from_input_id(&id),
                        message: log_str,
                        fields: BTreeMap::new(),
                    }
                };

                // Add source node if not present
                if !log.fields.contains_key("source_node") {
                    log.fields.insert(
                        "source_node".to_string(),
                        serde_json::Value::String(extract_node_from_input_id(&id))
                    );
                }

                // Add to buffer
                let timestamp_key = (log.timestamp * 1_000_000.0) as u64;
                log_buffer.insert(timestamp_key, log);

                // Flush logs older than 1 second (allow for ordering)
                let cutoff = timestamp_key - 1_000_000;
                let ready_logs: Vec<_> = log_buffer
                    .range(..cutoff)
                    .map(|(k, v)| (*k, v.clone()))
                    .collect();

                for (ts, log) in ready_logs {
                    // Send to unified output
                    let json = serde_json::to_string(&log)?;
                    node.send_output("unified", Default::default(), json)?;

                    // Send to alerts if error/warn
                    if log.level == "ERROR" || log.level == "WARN" {
                        node.send_output("alerts", Default::default(), json.clone())?;

                        // Send to webhook if configured
                        if let Ok(webhook) = std::env::var("ALERT_WEBHOOK") {
                            send_to_webhook(&webhook, &log)?;
                        }
                    }

                    // Remove from buffer
                    log_buffer.remove(&ts);
                }
            }
            Event::Stop => break,
            _ => {}
        }
    }

    // Flush remaining logs
    for (_, log) in log_buffer {
        let json = serde_json::to_string(&log)?;
        node.send_output("unified", Default::default(), json)?;
    }

    Ok(())
}
```

---

## API Design

### LogMessage Data Structure

**Enhanced structure** (extends current):

```rust
// libraries/message/src/common.rs
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LogMessage {
    // Existing fields
    pub build_id: Option<BuildId>,
    pub daemon_id: Option<DaemonId>,
    pub dataflow_id: Option<Uuid>,
    pub node_id: Option<NodeId>,
    pub level: LogLevel,
    pub target: Option<String>,
    pub module_path: Option<String>,
    pub file: Option<String>,
    pub line: Option<u32>,
    pub message: String,

    // New fields for structured logging
    pub timestamp: u64,  // Unix timestamp microseconds
    pub hlc: Option<uhlc::Timestamp>,  // HLC timestamp for distributed ordering
    pub fields: Option<HashMap<String, serde_json::Value>>,  // Custom fields
    pub trace_id: Option<String>,  // OpenTelemetry trace ID
    pub span_id: Option<String>,   // OpenTelemetry span ID
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum LogLevel {
    Trace = 0,
    Debug = 1,
    Info = 2,
    Warn = 3,
    Error = 4,
}
```

### Configuration

**Node-level configuration:**

```yaml
# dataflow.yml
nodes:
  - id: my_node
    path: ./my_node
    logging:
      # Enable dataflow-native logging
      dataflow_output: true  # Creates __dora_logs output

      # File logging config
      file:
        enabled: true
        level: info
        format: json  # or "pretty"

      # Real-time streaming
      streaming:
        enabled: true
        level: warn  # Only stream warn/error
```

**Global configuration:**

```yaml
# dataflow.yml
_unstable_logging:
  default_level: info
  format: json

  # Auto-create log aggregator
  aggregator:
    enabled: true
    outputs:
      - unified
      - alerts
```

---

## Data Formats

### JSON Log Format

```json
{
  "timestamp": 1704729600000000,
  "hlc": "1704729600000000-0-00000000000000000000",
  "level": "INFO",
  "node": "asr",
  "message": "Transcription complete",
  "fields": {
    "question_id": 485057,
    "language": "zh",
    "duration_ms": 1200,
    "confidence": 0.95
  },
  "trace_id": "4bf92f3577b34da6a3ce929d0e0e4736",
  "span_id": "00f067aa0ba902b7"
}
```

### Pretty Format (Human-Readable)

```
[2025-01-08T12:00:00.000000Z][INFO ][asr] Transcription complete
  question_id: 485057
  language: zh
  duration_ms: 1200
  confidence: 0.95
```

### Compact Format

```
2025-01-08T12:00:00Z asr INFO: Transcription complete (question_id=485057, language=zh)
```

### Arrow Format (Dataflow Transport)

Logs sent through dataflow use Arrow's struct array:

```rust
// Schema
Schema {
    fields: [
        Field { name: "timestamp", data_type: UInt64 },
        Field { name: "level", data_type: Utf8 },
        Field { name: "node", data_type: Utf8 },
        Field { name: "message", data_type: Utf8 },
        Field { name: "fields", data_type: Utf8 },  // JSON string
    ]
}
```

---

## Implementation Plan

### Phase 1: Foundation 

**Goals:**
- Add JSON format support to `dora-tracing`
- Expose existing `LogSubscribe` in CLI
- Document `send_stdout_as` pattern

**Tasks:**
- [ ] Add `format` parameter to `TracingBuilder`
  - `TracingBuilder::with_stdout("info").with_format(Format::Json)`
- [ ] Implement JSON formatter for `tracing_subscriber`
- [ ] Add `--follow` flag to `dora logs` CLI
- [ ] Write example: structured logging with `send_stdout_as`
- [ ] Update documentation

**Deliverables:**
- Users can do `DORA_LOG_FORMAT=json cargo run`
- `dora logs <dataflow> <node> --follow` works
- Example showing JSON logging pattern

### Phase 2: Cross-Language APIs 

**Goals:**
- Add logging APIs for Rust, Python, C/C++
- Emit to implicit `__dora_logs` output
- Maintain backward compatibility

**Tasks:**

**Rust API:**
- [ ] Create `apis/rust/node/src/logger.rs`
- [ ] Implement `DoraLogger` and `LogBuilder`
- [ ] Add `DoraNode::log()` method
- [ ] Tests for structured logging

**Python API:**
- [ ] Create PyO3 bindings in `apis/python/node/src/log.rs`
- [ ] Expose `node.log()` in Python
- [ ] Optional: Integrate with Python's `logging` module
- [ ] Python examples and tests

**C/C++ API:**
- [ ] Create `apis/c/node/src/log.h`
- [ ] Implement FFI functions
- [ ] C++ wrapper class
- [ ] Examples for C and C++

**Deliverables:**
- `node.log().info("message").field("key", value).send()` works in all languages
- Logs appear in both files and optional dataflow output
- Comprehensive tests

### Phase 3: CLI Enhancements 

**Goals:**
- Add filtering, formatting, and aggregation to CLI
- Unified timeline across nodes

**Tasks:**
- [ ] Add `--level`, `--format`, `--since`, `--until` flags
- [ ] Implement `--all-nodes` for unified timeline
- [ ] Add `--grep` filtering
- [ ] Parse structured logs for better filtering
- [ ] Improve output formatting (colors, table layout)

**Deliverables:**
- `dora logs --all-nodes --level error --format json`
- Merged timeline from multiple nodes sorted by HLC
- User guide for CLI features

### Phase 4: Log Aggregator 

**Goals:**
- Build log aggregator node
- Support external integrations

**Tasks:**
- [ ] Create `libraries/log-aggregator` crate
- [ ] Implement log collection and parsing
- [ ] Timeline merging with HLC
- [ ] Add filtering and routing logic
- [ ] Webhook integration for alerts
- [ ] Grafana Loki exporter

**Deliverables:**
- `dora-log-aggregator` binary
- Example dataflows using aggregator
- Integration guides (Loki, Datadog, etc.)

### Phase 5: Integrations & Polish 

**Goals:**
- OpenTelemetry integration
- Performance optimization
- Documentation

**Tasks:**
- [ ] Correlate logs with OTel traces
- [ ] Add metrics emission (log counts, etc.)
- [ ] Performance benchmarks
- [ ] Optimize for high-volume logging
- [ ] Complete documentation
- [ ] Migration guide

**Deliverables:**
- Logs linked to traces in Jaeger
- <1μs overhead for log emission
- Complete user documentation

---

## Performance Considerations

### Overhead Analysis

**Log Emission Overhead:**

| Operation | Target Latency | Notes |
|-----------|----------------|-------|
| `node.log().info("msg").send()` | <1μs | Fast path, no fields |
| With 5 fields | <5μs | Serialization cost |
| To dataflow output | +2-10μs | Arrow conversion + send |
| To file | +10-50μs | Async write, buffered |

**Memory:**
- Log buffer: ~1KB per log message
- Aggregator buffer: Configurable (default: 10MB, ~10k logs)

**Throughput:**
- Target: 100k logs/sec per node
- Aggregator: 500k logs/sec aggregated

### Optimization Strategies

1. **Lazy Evaluation:**
   ```rust
   // Don't serialize fields unless log level is enabled
   if log.level >= current_level {
       let json = serde_json::to_string(&fields)?;
       emit(json);
   }
   ```

2. **Batching:**
   ```rust
   // Batch multiple logs into single Arrow array
   let batch_size = 100;
   let mut log_batch = Vec::with_capacity(batch_size);
   // ... collect logs ...
   send_batch_to_output(log_batch)?;
   ```

3. **Async Logging:**
   ```rust
   // Use channel for async emission
   log_tx.send(log_message).await?;
   // Background task handles actual I/O
   ```

4. **Level-based Routing:**
   ```yaml
   logging:
     file:
       level: debug  # Everything to file
     dataflow:
       level: warn   # Only warn/error to dataflow
   ```

### Backpressure Handling

**Problem:** If log aggregator is slow, logs might back up

**Solutions:**

1. **Queue Size Limits:**
   ```yaml
   inputs:
     asr_logs:
       source: asr/logs
       queue_size: 1000  # Drop if full
   ```

2. **Sampling:**
   ```rust
   // Sample debug logs at 10%
   if log.level == Debug && rand::random::<f32>() > 0.1 {
       return; // Don't emit
   }
   ```

3. **Dynamic Level Adjustment:**
   ```rust
   // If queue >90% full, raise min level
   if queue.len() > queue.capacity() * 0.9 {
       current_level = Warn;
   }
   ```

---

## Security & Privacy

### Sensitive Data Handling

**Problem:** Logs may contain PII, credentials, etc.

**Solutions:**

1. **Field Redaction:**
   ```rust
   // Automatic redaction of sensitive fields
   const SENSITIVE_FIELDS: &[&str] = &["password", "api_key", "token"];

   for field in SENSITIVE_FIELDS {
       if fields.contains_key(field) {
           fields.insert(field, "[REDACTED]");
       }
   }
   ```

2. **Regex Redaction:**
   ```rust
   // Redact email addresses, credit cards, etc.
   let email_regex = Regex::new(r"[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}")?;
   message = email_regex.replace_all(&message, "[EMAIL]").to_string();
   ```

3. **Configuration:**
   ```yaml
   logging:
     redact:
       - password
       - api_key
       - user_email
     patterns:
       - regex: '\b\d{16}\b'
         replacement: '[CREDIT_CARD]'
   ```

### Access Control

**File Permissions:**
- Log files: 0600 (owner read/write only)
- Log directory: 0700 (owner access only)

**Network Security:**
- LogSubscribe over TLS
- Webhook authentication
- Loki/external exports with API keys

---

## Testing Strategy

### Unit Tests

**Logger Tests:**
```rust
#[test]
fn test_structured_logging() {
    let logger = DoraLogger::new_test();

    logger.info("test message")
        .field("key", "value")
        .field("number", 42)
        .send()
        .unwrap();

    let emitted = logger.get_emitted_logs();
    assert_eq!(emitted.len(), 1);
    assert_eq!(emitted[0].message, "test message");
    assert_eq!(emitted[0].fields["key"], "value");
}
```

**CLI Tests:**
```rust
#[test]
fn test_log_filtering() {
    let logs = vec![
        LogMessage { level: Info, message: "info".into(), ... },
        LogMessage { level: Error, message: "error".into(), ... },
    ];

    let filtered = filter_logs(&logs, Some(LogLevel::Error));
    assert_eq!(filtered.len(), 1);
    assert_eq!(filtered[0].message, "error");
}
```

### Integration Tests

**End-to-End Logging:**
```rust
#[test]
fn test_e2e_logging() {
    // Start dataflow with logging
    let dataflow = r#"
        nodes:
          - id: node1
            path: ./test_node
            send_stdout_as: logs
          - id: aggregator
            path: dora-log-aggregator
            inputs:
              logs: node1/logs
    "#;

    run_dataflow(dataflow)?;

    // Verify logs received by aggregator
    let aggregator_output = read_output("aggregator", "unified")?;
    assert!(aggregator_output.contains("test message"));
}
```

**CLI Tests:**
```bash
# tests/cli_logs.sh
#!/bin/bash

# Start dataflow
dora start test_dataflow.yml &
sleep 2

# Test various CLI commands
dora logs test_dataflow node1 > /tmp/logs.txt
grep "test message" /tmp/logs.txt

dora logs test_dataflow --all-nodes --format json > /tmp/logs.json
jq '.level == "ERROR"' /tmp/logs.json

# Cleanup
dora stop test_dataflow
```

### Performance Tests

**Throughput Benchmark:**
```rust
#[bench]
fn bench_log_emission(b: &mut Bencher) {
    let logger = DoraLogger::new_test();

    b.iter(|| {
        logger.info("benchmark message")
            .field("counter", 42)
            .send()
            .unwrap();
    });
}

// Target: >1M logs/sec
```

**Aggregator Benchmark:**
```rust
#[bench]
fn bench_aggregator_throughput(b: &mut Bencher) {
    let aggregator = LogAggregator::new();

    b.iter(|| {
        for i in 0..1000 {
            aggregator.process_log(generate_log(i));
        }
    });
}

// Target: >500k logs/sec
```

---

## Migration Guide

### For Existing Users

**Step 1: Update to new version**
```bash
cargo update dora-node-api
pip install --upgrade dora-rs
```

**Step 2: Migrate from print() to structured logging**

**Before:**
```python
print(f"[INFO] Processing question {question_id}")
```

**After:**
```python
node.log().info("Processing question", question_id=question_id)
```

**Step 3: Add send_stdout_as for dataflow routing (optional)**

```yaml
# Only if you want logs in dataflow
nodes:
  - id: my_node
    send_stdout_as: logs
```

**Step 4: Use enhanced CLI**

```bash
# Old
dora logs my_dataflow my_node

# New (with filtering)
dora logs my_dataflow my_node --level error --format json
```

### Backward Compatibility

**Guarantees:**
- Existing `print()` in Python nodes still works
- `send_stdout_as` behavior unchanged
- Log files still at `out/<dataflow_id>/log_<node>.txt`
- `dora logs` without flags works identically

**Breaking Changes:**
- None in Phase 1-3
- Phase 4+ may require opt-in for new features

---

## Alternatives Considered

### Alternative 1: Use Standard Logging Libraries

**Approach:** Encourage users to use standard libraries (Python `logging`, Rust `tracing`) directly

**Pros:**
- Familiar to developers
- No custom API to learn
- Ecosystem integrations

**Cons:**
- Hard to unify across languages
- No dataflow integration
- Can't leverage dora's HLC timestamps
- Missing structured fields in C/C++

**Decision:** Rejected. We provide a dora-specific API that wraps standard libraries.

### Alternative 2: Logs Only to Files

**Approach:** Keep current file-based system, enhance CLI only

**Pros:**
- Simple, proven approach
- No dataflow message overhead
- Easy to debug

**Cons:**
- Can't route logs through dataflow
- Hard to build real-time dashboards
- No structured processing

**Decision:** Rejected. We provide both file-based AND dataflow-native options.

### Alternative 3: External Sidecar for Aggregation

**Approach:** Run Fluentd/Vector/Promtail alongside dora daemon

**Pros:**
- Standard tools
- Rich integrations
- Mature

**Cons:**
- Extra dependency
- Not dora-native
- Complex configuration
- Can't leverage shared memory

**Decision:** Rejected for default, but supported as integration option.

### Alternative 4: Centralized Logging Service

**Approach:** All nodes send logs to central service (similar to ROS2's rosout)

**Pros:**
- Single source of truth
- Easy to query
- Automatic aggregation

**Cons:**
- Single point of failure
- Network overhead
- Scaling challenges
- Adds latency

**Decision:** Rejected. We use distributed approach with optional aggregator node.

---

## Open Questions

### Q1: Should logging API be sync or async?

**Options:**
- **Sync**: `node.log().info("msg").send()?;`
- **Async**: `node.log().info("msg").send().await?;`

**Trade-offs:**
- Sync: Easier API, may block
- Async: Non-blocking, but complicates usage

**Recommendation:** Sync API with async backend (channel-based emission)

### Q2: How to handle log volume in production?

**Concerns:**
- High-volume logging can overwhelm aggregator
- Storage costs for large dataflows

**Solutions:**
- Sampling (keep 10% of debug logs)
- Dynamic level adjustment
- Retention policies
- Compression

**Recommendation:** Provide sampling config, document best practices

### Q3: Should __dora_logs be automatic or opt-in?

**Options:**
1. **Automatic**: Every node gets `__dora_logs` output
2. **Opt-in**: Only with `send_stdout_as: logs`

**Trade-offs:**
- Automatic: Convenient, but might surprise users
- Opt-in: Explicit, but requires config

**Recommendation:** Start with opt-in, consider automatic in v2

### Q4: How to handle multi-line logs?

**Example:**
```python
# Stack trace spanning multiple lines
try:
    raise Exception("error")
except Exception:
    traceback.print_exc()  # Prints to stdout
```

**Solutions:**
1. Buffer until next log timestamp
2. Explicit multi-line marker
3. Capture stderr separately

**Recommendation:** Buffer stdout until flush or timeout (1 second)

### Q5: Integration with OpenTelemetry traces?

**Desired:**
```rust
// Logs automatically linked to current trace/span
node.log().info("Processing").send()?;
// → includes trace_id and span_id
```

**Challenges:**
- Need to propagate trace context
- Python/C++ bindings complexity

**Recommendation:** Phase 5 feature, use `tracing-opentelemetry` integration

---

## Appendices

### A. Wire Format Specification

**LogMessage over TCP:**
```
┌────────────────┬──────────────────────────┐
│ Length (4B)    │ JSON Payload             │
│ Big Endian     │ UTF-8 Encoded            │
└────────────────┴──────────────────────────┘
```

**LogMessage in Arrow:**
```
StructArray {
    fields: [
        ("timestamp", UInt64Array),
        ("level", DictionaryArray<Utf8>),
        ("node", Utf8Array),
        ("message", Utf8Array),
        ("fields", Utf8Array),  // JSON string
    ]
}
```

### B. Environment Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `DORA_LOG_LEVEL` | Minimum log level | `info` |
| `DORA_LOG_FORMAT` | Output format | `pretty` |
| `DORA_LOG_FILE` | Enable file logging | `true` |
| `DORA_LOG_DATAFLOW` | Enable dataflow output | `false` |
| `DORA_LOG_REDACT` | Comma-separated fields to redact | `` |

### C. Metrics

**Exposed Metrics:**
```
dora_logs_emitted_total{node="asr",level="info"}
dora_logs_dropped_total{node="asr",reason="queue_full"}
dora_logs_aggregated_total{level="error"}
dora_log_emission_duration_seconds{quantile="0.99"}
```

### D. References

- Rust `tracing` crate: https://docs.rs/tracing
- OpenTelemetry Logs: https://opentelemetry.io/docs/specs/otel/logs/
- ROS2 Logging: https://design.ros2.org/articles/logging.html
- Grafana Loki: https://grafana.com/docs/loki/latest/
- Apache Arrow: https://arrow.apache.org/

---

**End of Technical Design Specification**

**Next Steps:**
1. Review with team
2. Prototype Phase 1 features
3. Gather feedback from users
4. Iterate on design

**Document Version History:**
- v1.0 (2025-01-08): Initial draft
