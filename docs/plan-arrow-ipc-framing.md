# Arrow IPC Framing: Schema Negotiation & Type Safety

## Context

Adora transmits Arrow data as raw bytes + `ArrowTypeInfo` metadata (buffer offsets, data type, null count). There is no Arrow IPC framing — no schema, no field names, no dictionary encoding. Nodes must pre-agree on types via YAML annotations or prior knowledge.

**What breaks:**
- Sender upgrades Float32 -> Float64: receiver gets garbage (wrong byte alignment)
- Struct field order changes: silent data corruption
- No runtime type mismatch detection (unless `ADORA_RUNTIME_TYPE_CHECK` env var set)
- No schema evolution (adding fields requires full redeployment)
- No dynamic type discovery (receiver can't inspect incoming schema)

**What works well:**
- Zero-copy shmem for large messages (no serialization overhead)
- Minimal metadata overhead for high-frequency messages
- Type-optional ports (untyped nodes work fine)

## Current Data Flow

```
Sender:  Arrow Array -> copy buffers to flat bytes + ArrowTypeInfo
         -> shmem (>=4KB) or TCP Vec (<4KB) + metadata
Receiver: flat bytes + ArrowTypeInfo -> reconstruct Arrow Array
         (assumes pre-agreed type)
```

## Approach: Optional Arrow IPC Framing (Opt-In Per Output)

Don't change the default fast path. Add an optional `framing: arrow-ipc` flag on outputs that enables full Arrow IPC serialization with schema. This gives:
- Schema embedded in every message (self-describing)
- Runtime type validation on receive
- Dictionary encoding support
- Zero change for existing unframed outputs

## Phase 1: Runtime Type Validation (No Protocol Change)

**Goal:** Catch type mismatches at runtime without changing the wire format.

### 1.1 Always-on lightweight type check

Currently `ADORA_RUNTIME_TYPE_CHECK` is opt-in via env var. Make it automatic:
- On first message received per input, compare `ArrowTypeInfo.data_type` with `input_types` from descriptor
- Log warning on mismatch (don't error by default — backward compat)
- Zero cost after first message (cached result per input)

**Files:** `apis/rust/node/src/event_stream/mod.rs`, `apis/rust/node/src/node/mod.rs`

### 1.2 Type info in metadata for all messages

Currently `ArrowTypeInfo` is always sent. Enhance it:
- Add optional `field_names: Option<Vec<String>>` for struct types
- Add optional `schema_hash: Option<u64>` (hash of the full schema)
- Receiver can fast-check via hash (O(1)) before full validation

**File:** `libraries/message/src/metadata.rs`

## Phase 2: Arrow IPC Framing (Opt-In)

**Goal:** Self-describing messages with full Arrow IPC schema for interop scenarios.

### 2.1 YAML flag

```yaml
nodes:
  - id: camera
    outputs:
      - name: image
        framing: arrow-ipc  # opt-in full Arrow IPC
```

Default: `framing: raw` (current behavior).

**File:** `libraries/message/src/descriptor.rs`

### 2.2 IPC serialization path

When `framing: arrow-ipc`:
- Use `arrow::ipc::writer::StreamWriter` to serialize (includes schema + record batch)
- On receive, use `arrow::ipc::reader::StreamReader` to deserialize
- Schema is embedded — receiver can validate without prior knowledge

**Files:**
- `apis/rust/node/src/node/mod.rs` — `send_output` branches on framing mode
- `apis/rust/node/src/event_stream/data_conversion.rs` — `into_arrow_array` branches on framing

### 2.3 Shmem layout for IPC

For shmem path with IPC framing:
- Write Arrow IPC stream bytes to shmem region (instead of raw buffer)
- Slightly larger overhead (~100-200 bytes schema header per message)
- Still zero-copy for the payload bytes within the IPC stream

### 2.4 Python integration

PyArrow has native IPC support:
```python
# Send: pa.ipc.serialize(record_batch).to_buffer()
# Recv: pa.ipc.deserialize(buffer)
```

The Python API auto-detects framing mode from descriptor and uses the appropriate path.

**File:** `apis/python/node/src/lib.rs`

## Phase 3: Schema Evolution & Discovery

### 3.1 Schema registry (per-dataflow)

Store output schemas in coordinator when nodes register:
- Node reports its output schemas on Subscribe
- Coordinator stores in `RunningDataflow.output_schemas: BTreeMap<OutputId, Schema>`
- Downstream nodes can query schemas before consuming

### 3.2 Schema evolution rules

Define compatibility rules (similar to Avro/Protobuf):
- **Backward compatible**: add nullable field, widen numeric type
- **Breaking**: remove field, narrow type, reorder fields
- Coordinator validates on dynamic `AddMapping`

### 3.3 Type validation at dataflow startup

`adora validate` already checks type URN compatibility. Extend to:
- Resolve type URNs to full Arrow schemas
- Check field-level compatibility (not just top-level type name)
- Report field-level mismatches with suggestions

## Key Files

| File | Change |
|------|--------|
| `libraries/message/src/metadata.rs` | ArrowTypeInfo: add field_names, schema_hash |
| `libraries/message/src/descriptor.rs` | Output framing mode flag |
| `apis/rust/node/src/node/mod.rs` | send_output: branch on framing |
| `apis/rust/node/src/event_stream/data_conversion.rs` | Receive: branch on framing |
| `libraries/core/src/types.rs` | Schema-level compatibility checks |
| `libraries/core/src/descriptor/validate.rs` | Field-level validation |

## Performance Impact

| Mode | Overhead | Use Case |
|------|----------|----------|
| `framing: raw` (default) | 0 (current) | High-frequency sensors, benchmarks |
| `framing: arrow-ipc` | ~100-200 bytes/msg | Interop, debugging, schema evolution |
| Runtime type check (Phase 1) | ~1 comparison on first msg | Always-on safety net |

## Risks

- **Phase 2 IPC path not zero-copy**: Arrow IPC stream includes schema prefix per batch. For shmem, the data portion is still contiguous but needs offset adjustment. Performance regression for IPC-framed shmem paths (~5-10%).
- **Schema hash collisions**: u64 hash is not collision-free. Use as fast path only; full validation on hash mismatch.
- **Backward compat**: `framing: raw` is default; existing dataflows unchanged.

## Non-Goals

- Full schema registry service (like Confluent Schema Registry) — too heavy for robotics
- Protobuf/FlatBuffers support — Arrow is the canonical format
- Cross-language schema negotiation beyond Rust/Python/C++ — stick to Arrow FFI
