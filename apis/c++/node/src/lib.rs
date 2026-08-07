use std::{
    any::Any,
    collections::{BTreeMap, VecDeque},
    time::{Duration, Instant},
    vec,
};

use crate::ffi::MetadataValueType;

use chrono::DateTime;
#[cfg(any(feature = "ros2-bridge", test))]
use dora_node_api::merged::MergeExternal;
use dora_node_api::{
    self, Event, EventStream, Metadata as DoraMetadata,
    MetadataParameters as DoraMetadataParameters, Parameter as DoraParameter, PatternError,
    TryRecvError,
    arrow::array::{AsArray, UInt8Array},
    dora_core::config::NodeId,
    merged::MergedEvent,
};
use eyre::{Result as EyreResult, bail, eyre};
use serde::Serialize;
use serde_json::Value as JsonValue;

#[cfg(feature = "ros2-bridge")]
pub use prelude::*;
#[cfg(feature = "ros2-bridge")]
pub mod prelude {
    pub use dora_ros2_bridge::prelude::*;
}
use futures_lite::{Stream, StreamExt, stream};

#[cxx::bridge]
#[allow(clippy::needless_lifetimes)]
mod ffi {
    struct DoraNode {
        events: Box<Events>,
        send_output: Box<OutputSender>,
    }

    pub enum DoraEventType {
        Stop,
        Input,
        InputClosed,
        Error,
        Unknown,
        AllInputsClosed,
        /// Non-blocking poll (`try_next_event`) found no event ready,
        /// and the same value is returned by `drained_events_next` once
        /// a drained queue is exhausted. Distinct from `Timeout` —
        /// `Empty` is returned even when no timeout was set.
        Empty,
        /// `next_event_timeout` returned without an event because the
        /// caller-supplied deadline elapsed first.
        Timeout,
        /// An upstream node has failed. Use `event_as_node_failed` to
        /// extract the failed node id, the error message, and the list
        /// of downstream inputs that will stop receiving data.
        NodeFailed,
        /// Hot-reload notification for an operator. Operator id is
        /// available via the daemon's reload protocol; this variant
        /// exists so C++ nodes can react to reloads (e.g. flushing
        /// caches) rather than treating them as `Unknown`.
        Reload,
    }

    struct DoraInput {
        id: String,
        data: Vec<u8>,
    }

    /// `DoraInput` plus the input's metadata.
    ///
    /// The metadata carries the Service/Action correlation keys
    /// (`request_id`, `goal_id`, `goal_status`), so the server side of a
    /// request/reply exchange needs it to echo the correlation back.
    /// `event_as_arrow_input_with_info` also exposes metadata, but only
    /// by exporting the payload through the Arrow C Data Interface —
    /// which would force an Arrow C++ dependency on byte-payload nodes.
    struct DoraInputWithMetadata {
        id: String,
        data: Vec<u8>,
        metadata: Box<Metadata>,
    }

    struct DoraResult {
        error: String,
    }

    /// Terminal outcome of a pattern-aware wait
    /// (`recv_service_response` / `recv_action_result`).
    ///
    /// Mirrors `dora_node_api::PatternError`, plus `Matched` for the
    /// success case and `InvalidArgument` for arguments rejected before
    /// the wait even starts.
    ///
    /// The success variant is called `Matched` rather than the more
    /// obvious `None` or `Success` because X11's `X.h` defines *both*
    /// of those as macros (`#define None 0L`, `#define Success 0`). Any
    /// node pulling in X11 — directly or via OpenCV / Qt / GTK — would
    /// otherwise fail to compile against this header.
    enum DoraPatternStatus {
        /// No error: the `event` field holds the correlated reply.
        Matched,
        /// The deadline elapsed before a correlated reply arrived.
        Timeout,
        /// The expected server node restarted, so the in-flight
        /// correlation was orphaned. Retry against the new instance.
        ServerRestarted,
        /// The event stream ended (dataflow stopping) before the reply
        /// arrived. The terminal `Stop` event is still delivered to the
        /// caller's next `next_event`.
        StreamEnded,
        /// An upstream error event surfaced during the wait.
        StreamError,
        /// A caller-supplied argument was rejected (e.g. a malformed
        /// server node id). Nothing was awaited.
        InvalidArgument,
    }

    /// Result of `recv_service_response` / `recv_action_result`.
    ///
    /// On success `status == Matched`, `error` is empty and `event` is
    /// the correlated `Input` event. On failure `event` still carries a
    /// matching `DoraEventType` (`Timeout` / `AllInputsClosed` /
    /// `Empty`) so callers can branch on either field.
    struct DoraPatternResult {
        status: DoraPatternStatus,
        error: String,
        event: Box<DoraEvent>,
    }

    /// Result of `send_service_request` / `send_arrow_service_request`.
    ///
    /// On success `error` is empty and `request_id` holds the generated
    /// UUID v7 to pass to `recv_service_response`. On failure
    /// `request_id` is empty.
    struct DoraRequestId {
        request_id: String,
        error: String,
    }

    /// Payload of a `DoraEventType::NodeFailed` event.
    struct DoraNodeFailed {
        /// Inputs on this node that will stop receiving data because
        /// the upstream node failed.
        affected_input_ids: Vec<String>,
        /// Human-readable error message from the failed node.
        error: String,
        /// Id of the node that failed.
        source_node_id: String,
    }

    /// Everything needed to register a memory pool.
    ///
    /// `transport` is `"auto"` (or empty), `"shmem"` or `"unified"`. `"auto"`
    /// resolves to `"unified"` when `receiver_is_cuda`, otherwise `"shmem"`;
    /// the `DORA_MEMORY_POOL_TRANSPORT` environment variable overrides
    /// `"auto"` and nothing else. `"ipc"` is rejected — exporting a CUDA IPC
    /// handle would require CUDA inside this binding, and it does not work on
    /// integrated GPUs anyway.
    ///
    /// `dtype` and `shape` must both be non-empty: the daemon rejects a
    /// registration without them, and `shape` multiplied out by `dtype`'s
    /// element size must fit in `size`.
    struct DoraMemoryPoolSpec {
        id: String,
        size: usize,
        dtype: String,
        shape: Vec<usize>,
        transport: String,
        receiver_is_cuda: bool,
    }

    struct ArrowInputInfo {
        id: String,
        metadata: Box<Metadata>,
        error: String,
    }

    enum MetadataValueType {
        Bool,
        Integer,
        Float,
        String,
        ListInt,
        ListFloat,
        ListString,
        Timestamp,
    }

    pub struct CombinedEvents {
        events: Box<MergedEvents>,
    }

    pub struct CombinedEvent {
        event: Box<MergedDoraEvent>,
    }

    extern "Rust" {
        type Events;
        type OutputSender;
        type DoraEvent;
        type DrainedEvents;
        type MergedEvents;
        type MergedDoraEvent;
        type Metadata;

        fn init_dora_node() -> Result<DoraNode>;

        fn dora_events_into_combined(events: Box<Events>) -> CombinedEvents;
        fn empty_combined_events() -> CombinedEvents;
        fn next(self: &mut Events) -> Box<DoraEvent>;
        fn next_event(events: &mut Box<Events>) -> Box<DoraEvent>;
        /// Block up to `timeout_ms` milliseconds for the next event.
        /// Returns an event with `event_type == Timeout` if the deadline
        /// elapses before one arrives, or `AllInputsClosed` if the
        /// stream closed first.
        fn next_event_timeout(events: &mut Box<Events>, timeout_ms: u64) -> Box<DoraEvent>;
        /// Non-blocking poll. Returns an event with `event_type ==
        /// Empty` if no event is immediately available, or
        /// `AllInputsClosed` if the stream is closed.
        fn try_next_event(events: &mut Box<Events>) -> Box<DoraEvent>;
        /// True when the event queue is currently empty (no events
        /// buffered). Note this can race against the daemon producing a
        /// new event; treat it as a hint, not a guarantee.
        fn events_is_empty(events: &Box<Events>) -> bool;
        /// Take a snapshot of all currently-buffered events. Subsequent
        /// `next_event` / `try_next_event` calls see only events that
        /// arrive after this point.
        fn drain_events(events: &mut Box<Events>) -> Box<DrainedEvents>;
        fn drained_events_len(drained: &Box<DrainedEvents>) -> usize;
        /// Pop the next event from a drained snapshot. Returns
        /// `event_type == Empty` once the snapshot is exhausted.
        fn drained_events_next(drained: &mut Box<DrainedEvents>) -> Box<DoraEvent>;
        fn event_type(event: &Box<DoraEvent>) -> DoraEventType;
        fn event_as_input(event: Box<DoraEvent>) -> Result<DoraInput>;
        /// Like `event_as_input`, but also hands back the input's
        /// metadata. Required by the server side of the Service and
        /// Action patterns, which must read the incoming `request_id` /
        /// `goal_id` in order to correlate its reply.
        fn event_as_input_with_metadata(event: Box<DoraEvent>) -> Result<DoraInputWithMetadata>;
        /// Extract the failure payload from a `NodeFailed` event.
        fn event_as_node_failed(event: Box<DoraEvent>) -> Result<DoraNodeFailed>;
        /// Selectively close one or more of this node's outputs without
        /// shutting the whole node down. Subsequent downstream
        /// subscribers see the corresponding `InputClosed` event.
        fn close_outputs(
            output_sender: &mut Box<OutputSender>,
            output_ids: Vec<String>,
        ) -> DoraResult;
        /// Return this node's `NodeRunConfig` (inputs / outputs /
        /// metadata block from the dataflow descriptor) serialized as
        /// JSON. Useful for runtime introspection: the C++ node can
        /// reason about its own declared interface without re-parsing
        /// the dataflow yaml.
        fn node_config_json(output_sender: &Box<OutputSender>) -> Result<String>;
        /// Return the full `Descriptor` (the parsed dataflow yaml)
        /// serialized as JSON. Useful for introspecting peer nodes,
        /// listing all topics, etc. May fail if the daemon hasn't
        /// delivered the descriptor yet.
        ///
        /// **Caution:** the returned JSON includes any `env` blocks
        /// declared on nodes in the dataflow yaml. Inline literals
        /// (`API_KEY: "..."`) and host-environment substitutions
        /// (`API_KEY: "${HOST_VAR}"`, expanded at descriptor parse
        /// time per `libraries/message/src/descriptor.rs`'s
        /// `with_expand_envs`) both appear as plain strings in the
        /// output. Don't pipe this into shared logs or telemetry
        /// without sanitizing if your dataflow can carry secrets in
        /// env vars.
        fn dataflow_descriptor_json(output_sender: &Box<OutputSender>) -> Result<String>;
        fn send_output(
            output_sender: &mut Box<OutputSender>,
            id: String,
            data: &[u8],
        ) -> DoraResult;
        fn log_message(
            output_sender: &Box<OutputSender>,
            level: String,
            message: String,
        ) -> DoraResult;
        fn send_output_with_metadata(
            output_sender: &mut Box<OutputSender>,
            id: String,
            data: &[u8],
            metadata: Box<Metadata>,
        ) -> DoraResult;

        // -------------------------------------------------------------
        // Service (request/reply) and Action (goal/feedback/result)
        // -------------------------------------------------------------

        /// Generate a unique, time-ordered request id (UUID v7).
        /// Mirrors `dora_node_api::DoraNode::new_request_id`.
        fn new_request_id() -> String;
        /// Alias of `new_request_id` that reads better in action
        /// (goal/feedback/result) contexts.
        fn new_goal_id() -> String;

        /// The terminal `goal_status` value for a goal that completed
        /// successfully. Use with `Metadata::set_goal_status` instead of
        /// hardcoding the string.
        fn goal_status_succeeded() -> String;
        /// The terminal `goal_status` value for a goal the server gave
        /// up on.
        fn goal_status_aborted() -> String;
        /// The terminal `goal_status` value for a goal canceled by the
        /// client.
        fn goal_status_canceled() -> String;

        /// Send a service request, injecting a freshly generated
        /// `request_id` into `metadata`, and return that id. Any
        /// `request_id` already present in `metadata` is replaced.
        fn send_service_request(
            output_sender: &mut Box<OutputSender>,
            output_id: String,
            data: &[u8],
            metadata: Box<Metadata>,
        ) -> DoraRequestId;

        /// Arrow-payload variant of `send_service_request`. Consumes the
        /// Arrow C Data Interface structs behind `array_ptr` /
        /// `schema_ptr` exactly like `send_arrow_output` does.
        unsafe fn send_arrow_service_request(
            output_sender: &mut Box<OutputSender>,
            output_id: String,
            array_ptr: *mut u8,
            schema_ptr: *mut u8,
            metadata: Box<Metadata>,
        ) -> DoraRequestId;

        /// Send a service response. A semantic alias of
        /// `send_output_with_metadata`: the server is expected to pass
        /// the incoming request's metadata (which carries `request_id`)
        /// straight through so the client can correlate the reply.
        fn send_service_response(
            output_sender: &mut Box<OutputSender>,
            output_id: String,
            data: &[u8],
            metadata: Box<Metadata>,
        ) -> DoraResult;

        /// Block until the response carrying `request_id` arrives from
        /// `server_node_id`, or up to `timeout_ms`.
        ///
        /// Non-matching events that arrive during the wait are buffered
        /// and replayed by later `next_event` calls, so the caller's
        /// main event loop loses nothing. If `server_node_id` restarts
        /// while waiting, the wait ends early with `ServerRestarted`
        /// rather than hanging until the deadline.
        fn recv_service_response(
            events: &mut Box<Events>,
            request_id: &str,
            server_node_id: &str,
            timeout_ms: u64,
        ) -> DoraPatternResult;

        /// Block until a *terminal* result (`goal_status` one of
        /// `succeeded` / `aborted` / `canceled`) for `goal_id` arrives
        /// from `server_node_id`, or up to `timeout_ms`.
        ///
        /// Intermediate feedback messages (same `goal_id`, no terminal
        /// `goal_status`) are buffered for the caller's own event loop
        /// rather than returned here.
        fn recv_action_result(
            events: &mut Box<Events>,
            goal_id: &str,
            server_node_id: &str,
            timeout_ms: u64,
        ) -> DoraPatternResult;

        // -------------------------------------------------------------
        // Memory-pool transport (producer side)
        // -------------------------------------------------------------

        type DoraMemoryPool;

        /// Create a pool segment and register it with the daemon. The
        /// segment is unlinked again if the daemon refuses it, so a failed
        /// registration leaves nothing behind in `/dev/shm`.
        fn register_memory_pool(
            output_sender: &mut Box<OutputSender>,
            spec: DoraMemoryPoolSpec,
        ) -> Result<Box<DoraMemoryPool>>;

        /// Page-aligned mapping base and total mapped bytes — this pair, and
        /// only this pair, is what `cudaHostRegister` takes. The payload start
        /// is not page-aligned, so it must never be registered directly.
        fn pool_shm_base(pool: &Box<DoraMemoryPool>) -> u64;
        fn pool_segment_bytes(pool: &Box<DoraMemoryPool>) -> usize;

        /// Payload pointer and length, as an out-param predicate. Returns
        /// false — leaving both outputs untouched — for an `ipc` pool, whose
        /// payload is in device memory and is NOT in this mapping. A struct
        /// with a nullable pointer would invite dereferencing without the
        /// check; this shape does not.
        fn pool_payload(pool: &Box<DoraMemoryPool>, out_ptr: &mut u64, out_len: &mut usize)
        -> bool;
        /// Offset of the payload within the mapping, for callers that hold a
        /// *device* base from `cudaHostGetDevicePointer` and need to reach the
        /// payload through it. Returns false for an `ipc` pool.
        fn pool_payload_offset(pool: &Box<DoraMemoryPool>, out_offset: &mut usize) -> bool;
        /// Bytes of payload actually present in this mapping; 0 for `ipc`.
        /// THIS is the bound for any copy — never a product computed from
        /// `pool_shape`.
        ///
        /// The size the producer declared in the segment metadata is
        /// deliberately not exposed. It would carry no information here —
        /// `PoolSegment::create` allocates exactly the declared size, so on
        /// the producer side it equals this — while on a pool created
        /// elsewhere it describes device memory outside the mapping, which
        /// would make it and `pool_shm_base` a pointer/length pair that does
        /// not bound itself.
        fn pool_payload_len(pool: &Box<DoraMemoryPool>) -> usize;

        fn pool_id(pool: &Box<DoraMemoryPool>) -> String;
        fn pool_shm_name(pool: &Box<DoraMemoryPool>) -> String;
        /// The resolved transport: `"shmem"` or `"unified"` for a pool this
        /// node created.
        fn pool_transport(pool: &Box<DoraMemoryPool>) -> String;
        /// Advisory metadata for interpreting the buffer — NOT a bound. An
        /// unrecognized dtype is assumed to be 1 byte per element, so a
        /// product computed from these under-estimates. Size a view with
        /// `pool_payload_len`.
        fn pool_dtype(pool: &Box<DoraMemoryPool>) -> String;
        /// Advisory metadata, as `pool_dtype`. Not a bound.
        fn pool_shape(pool: &Box<DoraMemoryPool>) -> Vec<usize>;
        fn pool_ipc_present(pool: &Box<DoraMemoryPool>) -> bool;

        /// Open a write cycle. Fails on an `ipc` pool, or if a cycle is
        /// already open on this handle. **Prefer the `dora::PoolWriteGuard`
        /// RAII wrapper** (`dora/memory_pool.hpp`) over calling this directly:
        /// an early return or a thrown exception between begin and end leaves
        /// the generation odd permanently, which kills the pool for every
        /// reader until some later successful write, and nothing on the Rust
        /// side recovers it.
        fn pool_begin_write(pool: &mut Box<DoraMemoryPool>) -> DoraResult;
        /// Close a write cycle; a no-op when none is open. `ok = false` leaves
        /// the pool marked incomplete — every reader rejects it until the next
        /// successful write — because an in-place write has already destroyed
        /// the previous frame and there is nothing to roll back to.
        ///
        /// **Prefer `dora::PoolWriteGuard`** here too: the generation is stuck
        /// odd precisely when a path out of the cycle skipped this call, so a
        /// caller arriving from a stuck pool is looking at the wrong end of
        /// the problem. The guard calls this from a destructor, which no early
        /// return or throw can skip.
        fn pool_end_write(pool: &mut Box<DoraMemoryPool>, ok: bool);
        /// True when a cycle is open on this handle — the leak detector for a
        /// caller that suspects it missed an end.
        fn pool_write_in_progress(pool: &Box<DoraMemoryPool>) -> bool;
        /// Copy `data` into the pool under the seqlock. `data.len()` must
        /// equal `pool_payload_len` exactly: a short write would leave the
        /// previous frame's bytes in the tail and publish them as complete.
        ///
        /// This opens and closes its own write cycle, so it must not be
        /// called while a `dora::PoolWriteGuard` is alive on the same pool —
        /// the second cycle is refused. Fill in place through the guard, or
        /// copy through this; not both.
        fn write_memory_pool(pool: &mut Box<DoraMemoryPool>, data: &[u8]) -> DoraResult;

        /// Ask the daemon to release the pool. The daemon unlinks the segment
        /// and notifies every node that touched it; this binding deliberately
        /// does not unlink locally.
        fn free_memory_pool(
            output_sender: &mut Box<OutputSender>,
            pool: Box<DoraMemoryPool>,
        ) -> DoraResult;

        // -------------------------------------------------------------
        // Memory-pool transport (consumer side)
        // -------------------------------------------------------------

        type DoraMemoryPoolView;
        type DoraPoolRead;

        /// Map another node's pool for reading.
        ///
        /// The daemon is asked for the pool's metadata and the segment name
        /// comes from its reply — never from the pool id, which does not
        /// determine the name and would map a segment the daemon has no record
        /// of. Fails if the pool is unknown, has already been freed, or its
        /// segment cannot be opened.
        ///
        /// Unlike a pool this node created, a view may be **`ipc`**: a Python
        /// sender on a discrete GPU registers a header-only segment whose
        /// payload lives in device memory. Every accessor below that reaches
        /// bytes reports that case rather than handing out an address.
        fn read_memory_pool(
            output_sender: &mut Box<OutputSender>,
            pool_id: &str,
        ) -> Result<Box<DoraMemoryPoolView>>;

        /// False once any node has freed this pool and `take_freed_pools` has
        /// observed the daemon's notification. Every accessor that yields an
        /// address or bytes fails from that point on.
        fn view_is_alive(view: &Box<DoraMemoryPoolView>) -> bool;

        /// Page-aligned mapping base and total mapped bytes — this pair, and
        /// only this pair, is what `cudaHostRegister` takes; the payload start
        /// is not page-aligned. Returns false, leaving both outputs untouched,
        /// once the pool has been freed. Paired as an out-param predicate so
        /// the base cannot be taken without its bound, nor either of them from
        /// a dead view.
        fn view_mapping(
            view: &Box<DoraMemoryPoolView>,
            out_base: &mut u64,
            out_bytes: &mut usize,
        ) -> bool;

        /// Payload pointer and length, as an out-param predicate. Returns
        /// false — leaving both outputs untouched — for an `ipc` pool, whose
        /// payload is in device memory and is NOT in this mapping, and for a
        /// pool that has been freed. This is the only way to reach the bytes:
        /// the size the sender declared in the segment metadata is
        /// deliberately not exposed, because on an `ipc` pool it describes a
        /// device buffer megabytes larger than the whole mapping.
        fn view_payload(
            view: &Box<DoraMemoryPoolView>,
            out_ptr: &mut u64,
            out_len: &mut usize,
        ) -> bool;
        /// Offset of the payload within the mapping, for a caller holding a
        /// *device* base from `cudaHostGetDevicePointer` that needs to reach
        /// the payload through it. False for an `ipc` or freed pool.
        fn view_payload_offset(view: &Box<DoraMemoryPoolView>, out_offset: &mut usize) -> bool;
        /// Bytes of payload present in this mapping; 0 for an `ipc` or freed
        /// pool. THIS is the bound for any copy — never a product computed
        /// from `view_shape`.
        fn view_payload_len(view: &Box<DoraMemoryPoolView>) -> usize;

        /// The pool id this view was opened with.
        fn view_id(view: &Box<DoraMemoryPoolView>) -> String;
        /// The shared-memory segment name the daemon reported for the pool.
        fn view_shm_name(view: &Box<DoraMemoryPoolView>) -> String;
        /// `"shmem"`, `"unified"` or `"ipc"`. `"unified"` means the sender
        /// intends a CUDA receiver to map the segment with
        /// `cudaHostRegister`; `"ipc"` means there is no payload here at all.
        fn view_transport(view: &Box<DoraMemoryPoolView>) -> String;
        /// Advisory metadata for interpreting the buffer — NOT a bound. An
        /// unrecognized dtype is assumed to be 1 byte per element, so a
        /// product computed from these under-estimates the real extent. Size
        /// every copy and every view with `view_payload_len`.
        fn view_dtype(view: &Box<DoraMemoryPoolView>) -> String;
        /// Advisory metadata, as `view_dtype`. Not a bound.
        fn view_shape(view: &Box<DoraMemoryPoolView>) -> Vec<usize>;
        /// True when the segment carries a CUDA IPC handle instead of a
        /// payload.
        fn view_ipc_present(view: &Box<DoraMemoryPoolView>) -> bool;
        /// The 64-byte CUDA IPC handle for `cudaIpcOpenMemHandle`, from a pool
        /// the Python binding registered on a discrete GPU. Empty when there
        /// is none, and empty once the pool has been freed — the device buffer
        /// it names is gone by then.
        fn view_ipc_handle(view: &Box<DoraMemoryPoolView>) -> Vec<u8>;

        /// Copy the whole frame into `dst` under the seqlock, returning true
        /// only if the copy is intact.
        ///
        /// This is the read most consumers want: the generation is sampled,
        /// the payload copied and the generation re-checked entirely inside
        /// Rust, so there is no token to mishandle. False means **nothing was
        /// copied that may be used** — a writer was mid-frame (retry on the
        /// next event), or the pool is `ipc` or freed, or `dst` is not exactly
        /// `view_payload_len()` bytes. Size `dst` from `view_payload_len()`,
        /// or use `dora::try_read_pool` (`dora/memory_pool.hpp`) which does,
        /// and then a false can only mean the retryable case.
        fn view_try_read(view: &Box<DoraMemoryPoolView>, dst: &mut [u8]) -> bool;

        /// Open a zero-copy read by sampling the pool's generation.
        ///
        /// For a reader that consumes the payload in place — a CUDA kernel
        /// over the device alias, a `cv::Mat` header — instead of copying it.
        /// Read through `view_payload`, then call `view_read_valid` with this
        /// token and **discard whatever was computed** if it returns false.
        ///
        /// The token is opaque on purpose: it cannot be constructed,
        /// compared, or arrived at by arithmetic on this side of the bridge,
        /// and it is bound to the view that issued it. Prefer
        /// `dora::PoolReadGuard` (`dora/memory_pool.hpp`), which keeps the
        /// token, the pointer and the length together.
        fn view_begin_read(view: &Box<DoraMemoryPoolView>) -> Box<DoraPoolRead>;
        /// True when the payload read since `view_begin_read` is intact: no
        /// write started or finished in between, the pool is still alive, and
        /// the token came from this view. A token from a different view is
        /// rejected rather than compared — two views of the same segment share
        /// a generation word, so comparing it would accept a bracket that
        /// never enclosed the read.
        fn view_read_valid(view: &Box<DoraMemoryPoolView>, read: &Box<DoraPoolRead>) -> bool;

        /// Ids of pools freed by any node since the last call.
        ///
        /// Views of those pools are marked dead before this returns, so no
        /// accessor can hand out a pointer into a pool the daemon has
        /// released. The ids come back so a CUDA consumer can
        /// `cudaHostUnregister` the mappings it registered **before** dropping
        /// the views that hold them: Rust cannot do that on its behalf, having
        /// no idea which segments were handed to the driver. Drain this once
        /// per event-loop pass.
        fn take_freed_pools() -> Vec<String>;

        fn next(self: &mut CombinedEvents) -> CombinedEvent;

        fn is_dora(self: &CombinedEvent) -> bool;
        fn downcast_dora(event: CombinedEvent) -> Result<Box<DoraEvent>>;

        unsafe fn send_arrow_output(
            output_sender: &mut Box<OutputSender>,
            id: String,
            array_ptr: *mut u8,
            schema_ptr: *mut u8,
        ) -> DoraResult;

        #[cxx_name = "send_arrow_output"]
        unsafe fn send_arrow_output_with_metadata(
            output_sender: &mut Box<OutputSender>,
            id: String,
            array_ptr: *mut u8,
            schema_ptr: *mut u8,
            metadata: Box<Metadata>,
        ) -> DoraResult;

        unsafe fn event_as_arrow_input(
            event: Box<DoraEvent>,
            out_array: *mut u8,
            out_schema: *mut u8,
        ) -> DoraResult;

        unsafe fn event_as_arrow_input_with_info(
            event: Box<DoraEvent>,
            out_array: *mut u8,
            out_schema: *mut u8,
        ) -> ArrowInputInfo;

        fn new_metadata() -> Box<Metadata>;
        fn timestamp(self: &Metadata) -> u64;
        fn get_bool(self: &Metadata, key: &str) -> Result<bool>;
        fn get_float(self: &Metadata, key: &str) -> Result<f64>;
        fn get_int(self: &Metadata, key: &str) -> Result<i64>;
        fn get_str(self: &Metadata, key: &str) -> Result<String>;
        fn get_list_int(self: &Metadata, key: &str) -> Result<Vec<i64>>;
        fn get_list_float(self: &Metadata, key: &str) -> Result<Vec<f64>>;
        fn get_list_string(self: &Metadata, key: &str) -> Result<Vec<String>>;
        fn get_timestamp(self: &Metadata, key: &str) -> Result<i64>;
        fn get_json(self: &Metadata, key: &str) -> Result<String>;
        fn to_json(self: &Metadata) -> String;
        fn list_keys(self: &Metadata) -> Vec<String>;
        fn set_bool(self: &mut Metadata, key: &str, value: bool) -> Result<()>;
        fn set_int(self: &mut Metadata, key: &str, value: i64) -> Result<()>;
        fn set_float(self: &mut Metadata, key: &str, value: f64) -> Result<()>;
        fn set_string(self: &mut Metadata, key: &str, value: String) -> Result<()>;
        fn set_list_int(self: &mut Metadata, key: &str, value: Vec<i64>) -> Result<()>;
        fn set_list_float(self: &mut Metadata, key: &str, value: Vec<f64>) -> Result<()>;
        fn set_list_string(self: &mut Metadata, key: &str, value: Vec<String>) -> Result<()>;
        fn set_timestamp(self: &mut Metadata, key: &str, value: i64) -> Result<()>;
        #[cxx_name = "type"]
        fn value_type(self: &Metadata, key: &str) -> Result<MetadataValueType>;

        /// Typed accessors for the reserved correlation keys used by the
        /// Service and Action patterns. Equivalent to `set_string` /
        /// `get_str` with the well-known key name, but they keep the key
        /// spelling out of user code — a typo there silently breaks
        /// correlation instead of failing loudly.
        fn set_request_id(self: &mut Metadata, value: String) -> Result<()>;
        fn request_id(self: &Metadata) -> Result<String>;
        fn set_goal_id(self: &mut Metadata, value: String) -> Result<()>;
        fn goal_id(self: &Metadata) -> Result<String>;
        fn set_goal_status(self: &mut Metadata, value: String) -> Result<()>;
        fn goal_status(self: &Metadata) -> Result<String>;
    }
}

#[cfg(feature = "ros2-bridge")]
pub mod ros2 {
    // pub use dora_ros2_bridge::*;
    include!(env!("ROS2_BINDINGS_PATH"));
}

fn init_dora_node() -> eyre::Result<ffi::DoraNode> {
    let (node, events) = dora_node_api::DoraNode::init_from_env()?;
    let events = Events(events);
    let send_output = OutputSender(node);

    Ok(ffi::DoraNode {
        events: Box::new(events),
        send_output: Box::new(send_output),
    })
}

pub struct Events(EventStream);

impl Events {
    fn next(&mut self) -> Box<DoraEvent> {
        Box::new(DoraEvent(match self.0.recv() {
            Some(e) => EventOrReason::Event(e),
            None => EventOrReason::Closed,
        }))
    }
}

fn next_event(events: &mut Box<Events>) -> Box<DoraEvent> {
    events.next()
}

/// Block up to `timeout_ms` for the next event. Polls `try_recv` with
/// an `Instant`-based deadline (and a short sleep between polls) so we
/// can distinguish "timed out" from "stream closed" structurally —
/// `EventStream::recv_timeout` returns `None` for both cases and
/// matching on a wrapped `Event::Error(msg.contains("timed out"))`
/// would be fragile (#1409 review feedback).
fn next_event_timeout(events: &mut Box<Events>, timeout_ms: u64) -> Box<DoraEvent> {
    // `timeout_ms` arrives across the cxx::bridge as `u64`, so a C++
    // caller can supply values that would panic the underlying
    // `Instant + Duration` arithmetic on platforms with a bounded
    // Instant range. Use `checked_add` and treat overflow as
    // "effectively no deadline" -- keep polling indefinitely until
    // an event arrives or the stream closes. Never returns
    // `TimedOut` in the overflow case, matching the caller's
    // intent ("wait as long as needed").
    let deadline = Instant::now().checked_add(Duration::from_millis(timeout_ms));
    // 1 ms keeps the busy-wait cost negligible while bounding overshoot
    // of the caller-supplied deadline to ~1 ms in the worst case.
    let poll_interval = Duration::from_millis(1);
    loop {
        match events.0.try_recv() {
            Ok(event) => return Box::new(DoraEvent(EventOrReason::Event(event))),
            Err(TryRecvError::Closed) => return Box::new(DoraEvent(EventOrReason::Closed)),
            Err(TryRecvError::Empty) => match deadline {
                Some(d) => {
                    let now = Instant::now();
                    if now >= d {
                        return Box::new(DoraEvent(EventOrReason::TimedOut));
                    }
                    std::thread::sleep(std::cmp::min(d - now, poll_interval));
                }
                None => std::thread::sleep(poll_interval),
            },
        }
    }
}

/// Non-blocking single poll. Returns `EventOrReason::Empty` (surfaced
/// as `DoraEventType::Empty` on the C++ side) when no event is
/// available; distinct from a real timeout because no timeout was
/// requested.
fn try_next_event(events: &mut Box<Events>) -> Box<DoraEvent> {
    Box::new(DoraEvent(match events.0.try_recv() {
        Ok(event) => EventOrReason::Event(event),
        Err(TryRecvError::Empty) => EventOrReason::Empty,
        Err(TryRecvError::Closed) => EventOrReason::Closed,
    }))
}

#[allow(clippy::borrowed_box)] // signature dictated by cxx::bridge
fn events_is_empty(events: &Box<Events>) -> bool {
    events.0.is_empty()
}

/// Snapshot of buffered events at a point in time. Subsequent
/// receives on the `Events` stream see only events that arrive after
/// this snapshot.
pub struct DrainedEvents(VecDeque<Event>);

fn drain_events(events: &mut Box<Events>) -> Box<DrainedEvents> {
    let drained = events.0.drain().unwrap_or_default();
    Box::new(DrainedEvents(drained.into()))
}

#[allow(clippy::borrowed_box)] // signature dictated by cxx::bridge
fn drained_events_len(drained: &Box<DrainedEvents>) -> usize {
    drained.0.len()
}

fn drained_events_next(drained: &mut Box<DrainedEvents>) -> Box<DoraEvent> {
    Box::new(DoraEvent(match drained.0.pop_front() {
        Some(e) => EventOrReason::Event(e),
        // Drained snapshots cannot transition from non-empty to closed
        // (the snapshot is frozen at `drain_events` time), so an
        // exhausted drain is `Empty`, not `Closed`.
        None => EventOrReason::Empty,
    }))
}

fn dora_events_into_combined(events: Box<Events>) -> ffi::CombinedEvents {
    let events = events.0.map(MergedEvent::Dora);
    ffi::CombinedEvents {
        events: Box::new(MergedEvents {
            events: Some(Box::new(events)),
            #[cfg(any(feature = "ros2-bridge", test))]
            next_id: 1,
        }),
    }
}

fn empty_combined_events() -> ffi::CombinedEvents {
    ffi::CombinedEvents {
        events: Box::new(MergedEvents {
            events: Some(Box::new(stream::empty())),
            #[cfg(any(feature = "ros2-bridge", test))]
            next_id: 1,
        }),
    }
}

pub struct DoraEvent(EventOrReason);

/// Internal representation that lets `try_next_event` /
/// `next_event_timeout` / `drained_events_next` report "no event"
/// outcomes to C++ without overloading `Option<Event>` (#1409 review).
//
// `Event` is much larger than the unit variants (Closed/Empty/TimedOut),
// but this enum is always wrapped in `Box<DoraEvent>` at the FFI boundary
// so the size delta isn't paid per-stack-slot. Boxing the `Event` variant
// would add a second allocation on every event delivery for no benefit.
#[allow(clippy::large_enum_variant)]
pub(crate) enum EventOrReason {
    Event(Event),
    /// Source stream closed; surfaces as `DoraEventType::AllInputsClosed`.
    Closed,
    /// Non-blocking poll found no event ready, or a drained snapshot
    /// was exhausted. Surfaces as `DoraEventType::Empty`.
    Empty,
    /// `next_event_timeout` deadline elapsed before an event arrived.
    /// Surfaces as `DoraEventType::Timeout`.
    TimedOut,
}

fn event_type(event: &DoraEvent) -> ffi::DoraEventType {
    match &event.0 {
        EventOrReason::Event(event) => match event {
            Event::Stop(_) => ffi::DoraEventType::Stop,
            Event::Input { .. } => ffi::DoraEventType::Input,
            Event::InputClosed { .. } => ffi::DoraEventType::InputClosed,
            Event::Error(_) => ffi::DoraEventType::Error,
            Event::NodeFailed { .. } => ffi::DoraEventType::NodeFailed,
            Event::Reload { .. } => ffi::DoraEventType::Reload,
            _ => ffi::DoraEventType::Unknown,
        },
        EventOrReason::Closed => ffi::DoraEventType::AllInputsClosed,
        EventOrReason::Empty => ffi::DoraEventType::Empty,
        EventOrReason::TimedOut => ffi::DoraEventType::Timeout,
    }
}

// `Box<DoraEvent>` is mandated by the cxx bridge signature (ownership transfer
// across the FFI boundary), so the `boxed_local` lint is a false positive.
#[allow(clippy::boxed_local)]
fn event_as_input(event: Box<DoraEvent>) -> eyre::Result<ffi::DoraInput> {
    let EventOrReason::Event(Event::Input { id, data, .. }) = event.0 else {
        bail!("not an input event");
    };

    Ok(ffi::DoraInput {
        id: id.into(),
        data: input_bytes(&data)?,
    })
}

#[allow(clippy::boxed_local)] // `Box<DoraEvent>` is mandated by the cxx bridge signature.
fn event_as_input_with_metadata(event: Box<DoraEvent>) -> eyre::Result<ffi::DoraInputWithMetadata> {
    let EventOrReason::Event(Event::Input { id, metadata, data }) = event.0 else {
        bail!("not an input event");
    };

    Ok(ffi::DoraInputWithMetadata {
        id: id.into(),
        data: input_bytes(&data)?,
        metadata: Box::new(Metadata::from_dora(metadata)?),
    })
}

/// Decode a byte-payload input. The payload arrives as a self-describing
/// Arrow IPC stream, so the type is read from the array itself rather
/// than assumed.
fn input_bytes(data: &dora_node_api::ArrowData) -> eyre::Result<Vec<u8>> {
    match data.data_type() {
        dora_node_api::arrow::datatypes::DataType::UInt8 => {
            let array: &UInt8Array = data.as_primitive();
            Ok(array.values().to_vec())
        }
        dora_node_api::arrow::datatypes::DataType::Null => Ok(vec![]),
        other => {
            bail!(
                "unsupported input arrow type {other:?}; \
                 use event_as_arrow_input for typed access"
            );
        }
    }
}

#[allow(clippy::boxed_local)] // `Box<DoraEvent>` is mandated by the cxx bridge signature.
fn event_as_node_failed(event: Box<DoraEvent>) -> eyre::Result<ffi::DoraNodeFailed> {
    let EventOrReason::Event(Event::NodeFailed {
        affected_input_ids,
        error,
        source_node_id,
    }) = event.0
    else {
        bail!("not a NodeFailed event");
    };
    Ok(ffi::DoraNodeFailed {
        affected_input_ids: affected_input_ids
            .into_iter()
            .map(|id| id.to_string())
            .collect(),
        error,
        source_node_id: source_node_id.to_string(),
    })
}

/// Parse a caller-supplied output id via `FromStr` instead of the panicking
/// `From<String>`.
///
/// `DataId::from(String)` is documented as panicking on invalid characters
/// (see `libraries/message/src/id.rs`, `# Panics`). Calling it on a
/// caller-supplied string would mean a typo in a C++ string literal aborts
/// the whole node by unwinding across the `cxx::bridge`. Returning the error
/// as a `DoraResult.error` instead lets the C++ caller handle it.
fn parse_output_id(id: &str) -> Result<dora_node_api::dora_core::config::DataId, ffi::DoraResult> {
    id.parse().map_err(|e| ffi::DoraResult {
        error: format!("invalid output id '{id}': {e}"),
    })
}

fn close_outputs(
    output_sender: &mut Box<OutputSender>,
    output_ids: Vec<String>,
) -> ffi::DoraResult {
    let mut ids = Vec::with_capacity(output_ids.len());
    for id in output_ids {
        match parse_output_id(&id) {
            Ok(parsed) => ids.push(parsed),
            Err(err) => return err,
        }
    }
    match output_sender.0.close_outputs(ids) {
        Ok(()) => ffi::DoraResult {
            error: String::new(),
        },
        Err(err) => ffi::DoraResult {
            error: format!("{err:?}"),
        },
    }
}

#[allow(clippy::borrowed_box)] // signature dictated by cxx::bridge
fn node_config_json(output_sender: &Box<OutputSender>) -> eyre::Result<String> {
    serde_json::to_string(output_sender.0.node_config())
        .map_err(|e| eyre!("failed to serialize node config: {e}"))
}

#[allow(clippy::borrowed_box)] // signature dictated by cxx::bridge
fn dataflow_descriptor_json(output_sender: &Box<OutputSender>) -> eyre::Result<String> {
    let desc = output_sender.0.dataflow_descriptor()?;
    serde_json::to_string(desc).map_err(|e| eyre!("failed to serialize dataflow descriptor: {e}"))
}

#[allow(clippy::boxed_local)] // `Box<DoraEvent>` is mandated by the cxx bridge signature.
unsafe fn event_as_arrow_input(
    event: Box<DoraEvent>,
    out_array: *mut u8,
    out_schema: *mut u8,
) -> ffi::DoraResult {
    // Cast to Arrow FFI types
    let out_array = out_array as *mut arrow::ffi::FFI_ArrowArray;
    let out_schema = out_schema as *mut arrow::ffi::FFI_ArrowSchema;

    let EventOrReason::Event(Event::Input {
        id: _,
        metadata: _,
        data,
    }) = event.0
    else {
        return ffi::DoraResult {
            error: "Not an input event".to_string(),
        };
    };

    if out_array.is_null() || out_schema.is_null() {
        return ffi::DoraResult {
            error: "Received null output pointer".to_string(),
        };
    }

    let array_data = data.to_data();

    match arrow::ffi::to_ffi(&array_data) {
        Ok((ffi_array, ffi_schema)) => {
            unsafe {
                std::ptr::write(out_array, ffi_array);
                std::ptr::write(out_schema, ffi_schema);
            }
            ffi::DoraResult {
                error: String::new(),
            }
        }
        Err(e) => ffi::DoraResult {
            error: format!("Error exporting Arrow array to C++: {e:?}"),
        },
    }
}

pub struct Metadata {
    timestamp: u64,
    parameters: BTreeMap<String, DoraParameter>,
}

impl Metadata {
    fn from_dora(metadata: DoraMetadata) -> EyreResult<Self> {
        Ok(Self {
            timestamp: metadata.timestamp().get_time().as_u64(),
            parameters: metadata.parameters,
        })
    }

    fn empty() -> Self {
        Self {
            timestamp: 0,
            parameters: BTreeMap::new(),
        }
    }

    fn parameter_type_name(parameter: &DoraParameter) -> &'static str {
        match parameter {
            DoraParameter::Bool(_) => "bool",
            DoraParameter::Integer(_) => "integer",
            DoraParameter::String(_) => "string",
            DoraParameter::Float(_) => "float",
            DoraParameter::ListInt(_) => "list<int>",
            DoraParameter::ListFloat(_) => "list<float>",
            DoraParameter::ListString(_) => "list<string>",
            DoraParameter::Timestamp(_) => "timestamp",
        }
    }

    fn expect_parameter<'a>(&'a self, key: &str) -> EyreResult<&'a DoraParameter> {
        self.parameters
            .get(key)
            .ok_or_else(|| eyre!("metadata missing key '{key}'"))
    }

    fn parameter_to_json(parameter: &DoraParameter, _key: &str) -> EyreResult<JsonValue> {
        match parameter {
            DoraParameter::Bool(value) => Ok(JsonValue::Bool(*value)),
            DoraParameter::Integer(value) => Ok(JsonValue::from(*value)),
            DoraParameter::Float(value) => Ok(JsonValue::from(*value)),
            DoraParameter::String(value) => Ok(JsonValue::String(value.clone())),
            DoraParameter::ListInt(values) => Ok(JsonValue::Array(
                values.iter().map(|value| JsonValue::from(*value)).collect(),
            )),
            DoraParameter::ListFloat(values) => Ok(JsonValue::Array(
                values.iter().map(|value| JsonValue::from(*value)).collect(),
            )),
            DoraParameter::ListString(values) => Ok(JsonValue::Array(
                values
                    .iter()
                    .map(|value| JsonValue::String(value.clone()))
                    .collect(),
            )),
            DoraParameter::Timestamp(dt) => {
                serde_json::to_value(dt).map_err(|e| eyre!("failed to serialize timestamp: {e}"))
            }
        }
    }

    pub fn timestamp(&self) -> u64 {
        self.timestamp
    }

    pub fn get_bool(&self, key: &str) -> EyreResult<bool> {
        let parameter = self.expect_parameter(key)?;
        match parameter {
            DoraParameter::Bool(value) => Ok(*value),
            other => Err(eyre!(
                "metadata key '{key}' has type '{}', expected 'bool'",
                Metadata::parameter_type_name(other)
            )),
        }
    }

    pub fn get_float(&self, key: &str) -> EyreResult<f64> {
        let parameter = self.expect_parameter(key)?;
        match parameter {
            DoraParameter::Float(value) => Ok(*value),
            other => Err(eyre!(
                "metadata key '{key}' has type '{}', expected 'float'",
                Metadata::parameter_type_name(other)
            )),
        }
    }

    pub fn get_int(&self, key: &str) -> EyreResult<i64> {
        let parameter = self.expect_parameter(key)?;
        match parameter {
            DoraParameter::Integer(value) => Ok(*value),
            other => Err(eyre!(
                "metadata key '{key}' has type '{}', expected 'integer'",
                Metadata::parameter_type_name(other)
            )),
        }
    }

    pub fn get_str(&self, key: &str) -> EyreResult<String> {
        let parameter = self.expect_parameter(key)?;
        match parameter {
            DoraParameter::String(value) => Ok(value.clone()),
            other => Err(eyre!(
                "metadata key '{key}' has type '{}', expected 'string'",
                Metadata::parameter_type_name(other)
            )),
        }
    }

    pub fn get_list_int(&self, key: &str) -> EyreResult<Vec<i64>> {
        let parameter = self.expect_parameter(key)?;
        match parameter {
            DoraParameter::ListInt(values) => Ok(values.clone()),
            other => Err(eyre!(
                "metadata key '{key}' has type '{}', expected 'list<int>'",
                Metadata::parameter_type_name(other)
            )),
        }
    }

    pub fn get_list_float(&self, key: &str) -> EyreResult<Vec<f64>> {
        let parameter = self.expect_parameter(key)?;
        match parameter {
            DoraParameter::ListFloat(values) => Ok(values.clone()),
            other => Err(eyre!(
                "metadata key '{key}' has type '{}', expected 'list<float>'",
                Metadata::parameter_type_name(other)
            )),
        }
    }

    pub fn get_list_string(&self, key: &str) -> EyreResult<Vec<String>> {
        let parameter = self.expect_parameter(key)?;
        match parameter {
            DoraParameter::ListString(values) => Ok(values.clone()),
            other => Err(eyre!(
                "metadata key '{key}' has type '{}', expected 'list<string>'",
                Metadata::parameter_type_name(other)
            )),
        }
    }

    pub fn get_timestamp(&self, key: &str) -> EyreResult<i64> {
        let parameter = self.expect_parameter(key)?;
        match parameter {
            DoraParameter::Timestamp(dt) => {
                // Convert chrono::DateTime<Utc> to nanoseconds since Unix epoch
                dt.timestamp_nanos_opt()
                    .ok_or_else(|| eyre!("Timestamp out of range for conversion to nanoseconds"))
            }
            other => Err(eyre!(
                "metadata key '{key}' has type '{}', expected 'timestamp'",
                Metadata::parameter_type_name(other)
            )),
        }
    }

    pub fn get_json(&self, key: &str) -> EyreResult<String> {
        let parameter = self.expect_parameter(key)?;
        let json_value = Metadata::parameter_to_json(parameter, key)?;
        serde_json::to_string(&json_value)
            .map_err(|err| eyre!("failed to serialize metadata value '{key}' to JSON: {err}"))
    }

    pub fn to_json(&self) -> String {
        #[derive(Serialize)]
        struct MetadataJson<'a> {
            timestamp: u64,
            parameters: &'a BTreeMap<String, DoraParameter>,
        }

        serde_json::to_string(&MetadataJson {
            timestamp: self.timestamp,
            parameters: &self.parameters,
        })
        .expect("failed to serialize metadata to JSON")
    }

    pub fn list_keys(&self) -> Vec<String> {
        self.parameters.keys().cloned().collect()
    }

    pub fn set_bool(&mut self, key: &str, value: bool) -> EyreResult<()> {
        self.insert_parameter(key, DoraParameter::Bool(value))
    }

    pub fn set_int(&mut self, key: &str, value: i64) -> EyreResult<()> {
        self.insert_parameter(key, DoraParameter::Integer(value))
    }

    pub fn set_float(&mut self, key: &str, value: f64) -> EyreResult<()> {
        self.insert_parameter(key, DoraParameter::Float(value))
    }

    pub fn set_string(&mut self, key: &str, value: String) -> EyreResult<()> {
        self.insert_parameter(key, DoraParameter::String(value))
    }

    pub fn set_list_int(&mut self, key: &str, value: Vec<i64>) -> EyreResult<()> {
        self.insert_parameter(key, DoraParameter::ListInt(value))
    }

    pub fn set_list_float(&mut self, key: &str, value: Vec<f64>) -> EyreResult<()> {
        self.insert_parameter(key, DoraParameter::ListFloat(value))
    }

    pub fn set_list_string(&mut self, key: &str, value: Vec<String>) -> EyreResult<()> {
        self.insert_parameter(key, DoraParameter::ListString(value))
    }

    pub fn set_timestamp(&mut self, key: &str, value: i64) -> EyreResult<()> {
        // Convert nanoseconds since Unix epoch to chrono::DateTime<Utc>.
        //
        // Use Euclidean division so pre-epoch (negative) timestamps split
        // correctly. Plain `/` and `%` truncate toward zero, yielding a
        // *negative* remainder for negative inputs; casting that to `u32`
        // wraps it to a huge value and makes `from_timestamp` reject the
        // (otherwise valid) instant. `div_euclid`/`rem_euclid` keep the
        // remainder in `0..1_000_000_000`, matching how `from_timestamp`
        // interprets `(secs, subsec_nanos)` and round-tripping with
        // `get_timestamp` for negative values.
        let secs = value.div_euclid(1_000_000_000);
        let subsec_nanos = value.rem_euclid(1_000_000_000) as u32;

        let dt = DateTime::from_timestamp(secs, subsec_nanos)
            .ok_or_else(|| eyre!("Invalid timestamp: out of range (nanos: {value})"))?;

        self.insert_parameter(key, DoraParameter::Timestamp(dt))
    }

    pub fn value_type(&self, key: &str) -> EyreResult<MetadataValueType> {
        let parameter = self.expect_parameter(key)?;
        let value_type = match parameter {
            DoraParameter::Bool(_) => MetadataValueType::Bool,
            DoraParameter::Integer(_) => MetadataValueType::Integer,
            DoraParameter::Float(_) => MetadataValueType::Float,
            DoraParameter::String(_) => MetadataValueType::String,
            DoraParameter::ListInt(_) => MetadataValueType::ListInt,
            DoraParameter::ListFloat(_) => MetadataValueType::ListFloat,
            DoraParameter::ListString(_) => MetadataValueType::ListString,
            DoraParameter::Timestamp(_) => MetadataValueType::Timestamp,
        };
        Ok(value_type)
    }

    pub fn set_request_id(&mut self, value: String) -> EyreResult<()> {
        self.set_string(dora_node_api::REQUEST_ID, value)
    }

    pub fn request_id(&self) -> EyreResult<String> {
        self.get_str(dora_node_api::REQUEST_ID)
    }

    pub fn set_goal_id(&mut self, value: String) -> EyreResult<()> {
        self.set_string(dora_node_api::GOAL_ID, value)
    }

    pub fn goal_id(&self) -> EyreResult<String> {
        self.get_str(dora_node_api::GOAL_ID)
    }

    pub fn set_goal_status(&mut self, value: String) -> EyreResult<()> {
        self.set_string(dora_node_api::GOAL_STATUS, value)
    }

    pub fn goal_status(&self) -> EyreResult<String> {
        self.get_str(dora_node_api::GOAL_STATUS)
    }

    fn into_parameters(self) -> DoraMetadataParameters {
        self.parameters
    }

    fn insert_parameter(&mut self, key: &str, parameter: DoraParameter) -> EyreResult<()> {
        self.parameters.insert(key.to_string(), parameter);
        Ok(())
    }
}

#[allow(clippy::boxed_local)] // `Box<DoraEvent>` is mandated by the cxx bridge signature.
unsafe fn event_as_arrow_input_with_info(
    event: Box<DoraEvent>,
    out_array: *mut u8,
    out_schema: *mut u8,
) -> ffi::ArrowInputInfo {
    // Cast to Arrow FFI types
    let out_array = out_array as *mut arrow::ffi::FFI_ArrowArray;
    let out_schema = out_schema as *mut arrow::ffi::FFI_ArrowSchema;

    let EventOrReason::Event(Event::Input { id, metadata, data }) = event.0 else {
        return ffi::ArrowInputInfo {
            id: String::new(),
            metadata: Box::new(Metadata::empty()),
            error: "Not an input event".to_string(),
        };
    };

    if out_array.is_null() || out_schema.is_null() {
        return ffi::ArrowInputInfo {
            id: id.to_string(),
            metadata: Box::new(Metadata::empty()),
            error: "Received null output pointer".to_string(),
        };
    }

    let prepared_metadata = match Metadata::from_dora(metadata) {
        Ok(metadata) => metadata,
        Err(err) => {
            return ffi::ArrowInputInfo {
                id: id.to_string(),
                metadata: Box::new(Metadata::empty()),
                error: format!("Error preparing metadata: {err:?}"),
            };
        }
    };

    let array_data = data.to_data();

    match arrow::ffi::to_ffi(&array_data) {
        Ok((ffi_array, ffi_schema)) => {
            unsafe {
                std::ptr::write(out_array, ffi_array);
                std::ptr::write(out_schema, ffi_schema);
            }
            ffi::ArrowInputInfo {
                id: id.to_string(),
                metadata: Box::new(prepared_metadata),
                error: String::new(),
            }
        }
        Err(e) => ffi::ArrowInputInfo {
            id: id.to_string(),
            metadata: Box::new(Metadata::empty()),
            error: format!("Error exporting Arrow array to C++: {e:?}"),
        },
    }
}

pub struct OutputSender(dora_node_api::DoraNode);

fn send_output(sender: &mut Box<OutputSender>, id: String, data: &[u8]) -> ffi::DoraResult {
    send_output_internal(sender, id, data, Default::default())
}

#[allow(clippy::borrowed_box)] // signature dictated by cxx::bridge
fn log_message(sender: &Box<OutputSender>, level: String, message: String) -> ffi::DoraResult {
    sender.0.log(&level, &message, None);
    ffi::DoraResult {
        error: String::new(),
    }
}

#[allow(clippy::boxed_local)]
fn send_output_with_metadata(
    sender: &mut Box<OutputSender>,
    id: String,
    data: &[u8],
    metadata: Box<Metadata>,
) -> ffi::DoraResult {
    let metadata = *metadata;
    let parameters = metadata.into_parameters();
    send_output_internal(sender, id, data, parameters)
}

fn send_output_internal(
    sender: &mut Box<OutputSender>,
    id: String,
    data: &[u8],
    metadata: DoraMetadataParameters,
) -> ffi::DoraResult {
    let output_id = match parse_output_id(&id) {
        Ok(parsed) => parsed,
        Err(err) => return err,
    };
    let result = sender
        .0
        .send_output_raw(output_id, metadata, data.len(), |out| {
            out.copy_from_slice(data)
        });
    let error = match result {
        Ok(()) => String::new(),
        Err(err) => format!("{err:?}"),
    };
    ffi::DoraResult { error }
}

// ---------------------------------------------------------------------
// Memory-pool transport (producer side)
// ---------------------------------------------------------------------

use dora_memory_pool::{
    naming,
    segment::{PoolSegment, Transport},
};

/// A pool this node owns: the mapped segment plus the id the daemon knows it
/// by. Dropping it unmaps the segment; only `free_memory_pool` releases it.
pub struct DoraMemoryPool {
    segment: PoolSegment,
    id: String,
}

const TRANSPORT_ENV: &str = "DORA_MEMORY_POOL_TRANSPORT";

/// Resolve a requested transport into a concrete mode, given the value of
/// [`TRANSPORT_ENV`] (`None` when it is unset).
///
/// `auto` never resolves to a mode that would fail at registration: `unified`
/// works everywhere (on a discrete GPU it is merely slower than IPC), and on
/// an integrated GPU it is the only thing that works at all.
///
/// The environment value is a parameter rather than a read, so the decision is
/// a pure function. The alternative — tests that set and unset a process-global
/// variable other threads are reading — is what Rust 2024 made `unsafe`, and
/// it makes the test outcome depend on the harness's thread count.
fn resolve_transport_with(
    requested: &str,
    receiver_is_cuda: bool,
    env_override: Option<&str>,
) -> EyreResult<Transport> {
    // An unset field on the C++ side arrives as an empty string, which means
    // "no preference", not "unknown transport".
    let requested = if requested.is_empty() {
        "auto"
    } else {
        requested
    };
    let resolved = match requested {
        // The override applies to `auto` only: an explicit request is the node
        // author's decision about its own buffer layout, and an operator's
        // environment must not silently rewrite it.
        "auto" => match env_override {
            None if receiver_is_cuda => Transport::Unified,
            None => Transport::Shmem,
            Some("shmem") => Transport::Shmem,
            Some("unified") => Transport::Unified,
            Some(other) => bail!(
                "{TRANSPORT_ENV}=`{other}` is not valid; expected `shmem` or `unified`. \
                 Ignoring it would hand back the transport the variable was set to change."
            ),
        },
        "shmem" => Transport::Shmem,
        "unified" => Transport::Unified,
        "ipc" => bail!(
            "transport `ipc` is not supported by the C++ binding: exporting a CUDA IPC handle \
             would require CUDA inside the binding, and cudaIpcGetMemHandle is unsupported on \
             integrated GPUs. Use `unified`."
        ),
        other => bail!("unknown transport `{other}`; expected `auto`, `shmem` or `unified`"),
    };
    Ok(resolved)
}

fn resolve_transport(requested: &str, receiver_is_cuda: bool) -> EyreResult<Transport> {
    let env_override = match std::env::var(TRANSPORT_ENV) {
        Ok(value) => Some(value),
        Err(std::env::VarError::NotPresent) => None,
        // Reported rather than swallowed, for the same reason an unrecognized
        // value is: a variable that was set and had no effect is worse than a
        // failure that names it.
        Err(std::env::VarError::NotUnicode(value)) => bail!(
            "{TRANSPORT_ENV} is not valid UTF-8 (`{}`); expected `shmem` or `unified`",
            value.to_string_lossy()
        ),
    };
    resolve_transport_with(requested, receiver_is_cuda, env_override.as_deref())
}

/// Reject a spec the daemon will refuse, before a segment exists for it.
///
/// `RegisterPinnedMemory` (`binaries/daemon/src/lib.rs`) rejects an empty
/// `dtype` or `shape`. Every other field it validates is already guaranteed by
/// the time we get there — `PoolSegment::create` bounds the size, `segment_name`
/// builds a name that passes the daemon's namespace check — so these two are
/// the only ones that would otherwise cost a create-and-unlink round trip to
/// discover, and come back as a bare "missing shape".
fn validate_daemon_required_fields(dtype: &str, shape: &[usize]) -> EyreResult<()> {
    if dtype.is_empty() {
        bail!("dtype must not be empty: the daemon rejects a pool registration without one");
    }
    if shape.is_empty() {
        bail!("shape must not be empty: the daemon rejects a pool registration without one");
    }
    Ok(())
}

/// Fill the daemon's pool parameters, mirroring `apis/python/node/src/lib.rs` —
/// the daemon validates these keys and the Python binding is the reference for
/// what it expects.
///
/// Pure, and separate from the send, so the key names and parameter variants
/// can be pinned by a test: every one of them is a string the daemon looks up
/// by hand, and a typo in any of them fails only at runtime, as a bare
/// "missing shared_memory_name".
fn pool_daemon_params(
    payload_ptr: u64,
    spec: &ffi::DoraMemoryPoolSpec,
    name: &str,
    transport: Transport,
) -> DoraMetadataParameters {
    let mut params = DoraMetadataParameters::new();
    // Valid only in this process; a consumer reaches the payload through
    // `shared_memory_name`, never through this.
    params.insert(
        "ptr".to_string(),
        DoraParameter::Integer(payload_ptr as i64),
    );
    params.insert("size".to_string(), DoraParameter::Integer(spec.size as i64));
    params.insert(
        "dtype".to_string(),
        DoraParameter::String(spec.dtype.clone()),
    );
    params.insert(
        "shape".to_string(),
        DoraParameter::ListInt(spec.shape.iter().map(|&dim| dim as i64).collect()),
    );
    params.insert(
        "shared_memory_name".to_string(),
        DoraParameter::String(name.to_string()),
    );
    // The binding never calls `cudaHostRegister` itself — that is the C++
    // side's job via `dora/cuda_pool.hpp` — so the pool is not pinned as far
    // as the daemon's bookkeeping is concerned.
    params.insert("is_pinned".to_string(), DoraParameter::Bool(false));
    params.insert(
        "pinned_type".to_string(),
        DoraParameter::String(transport.pinned_type().to_string()),
    );
    // No IPC handle is ever written to a segment this binding creates, so the
    // data region really is the payload.
    params.insert("ipc_present".to_string(), DoraParameter::Bool(false));
    params.insert(
        "buffer_id".to_string(),
        DoraParameter::String(spec.id.clone()),
    );
    params
}

fn register_with_daemon(
    output_sender: &mut Box<OutputSender>,
    segment: &PoolSegment,
    spec: &ffi::DoraMemoryPoolSpec,
    name: &str,
    transport: Transport,
) -> EyreResult<()> {
    // The daemon rejects `ptr == 0`. This is always `Some` here — `create`
    // refuses the one transport that has no payload in its mapping — but going
    // through the accessor keeps the one-past-the-end address of an `ipc`
    // segment unreachable from this file too.
    let (payload_ptr, _) = segment
        .payload()
        .ok_or_else(|| eyre!("pool `{}` has no payload to register", spec.id))?;

    let params = pool_daemon_params(payload_ptr, spec, name, transport);
    let timestamp = dora_node_api::uhlc::HLC::default().new_timestamp();
    let metadata = DoraMetadata::from_parameters(timestamp, params);
    output_sender
        .0
        .register_pinned_memory(spec.id.clone(), metadata)
}

/// True where [`naming::unlink_segment`] can actually remove a segment. Off
/// Linux it fails for every name with the same "unavailable on this platform"
/// message, which says nothing about the segment that was just created.
const UNLINK_SUPPORTED: bool = cfg!(target_os = "linux");

/// The clause appended to a failed registration describing what happened to the
/// segment it created. `unlink_supported` is a parameter so both branches are
/// reachable from a test on any host.
fn cleanup_clause(unlink: Result<(), String>, unlink_supported: bool) -> String {
    match unlink {
        Ok(()) => String::new(),
        // Reporting "additionally failed to remove the orphaned segment" where
        // unlinking was never applicable blames the cleanup for a platform
        // limitation and pushes the registration error — the one the caller
        // has to act on — behind noise.
        Err(_) if !unlink_supported => String::new(),
        Err(err) => format!("; additionally failed to remove the orphaned segment: {err}"),
    }
}

/// Create a pool segment and hand it to the daemon.
///
/// Every error on this path must be returned, never panicked: cxx wraps each
/// `extern "Rust"` shim in `prevent_unwind`, which calls `std::abort` rather
/// than converting a panic into a C++ exception. A panic between
/// `PoolSegment::create` and a successful registration therefore skips the
/// unlink below and strands the segment in `/dev/shm` — reclaimed only by the
/// daemon's orphan sweep at the next start of the same dataflow. Nothing here
/// can panic today; the constraint is on what may be added.
fn register_memory_pool(
    output_sender: &mut Box<OutputSender>,
    spec: ffi::DoraMemoryPoolSpec,
) -> EyreResult<Box<DoraMemoryPool>> {
    let transport = resolve_transport(&spec.transport, spec.receiver_is_cuda)?;
    validate_daemon_required_fields(&spec.dtype, &spec.shape)?;

    let dataflow_id = output_sender.0.dataflow_id().to_string();
    let node_id = output_sender.0.id().to_string();
    let name = naming::segment_name(&dataflow_id, &node_id, &spec.id).map_err(|e| eyre!("{e}"))?;

    let segment = PoolSegment::create(&name, spec.size, &spec.dtype, &spec.shape, transport)
        .map_err(|e| eyre!("{e}"))?;

    match register_with_daemon(output_sender, &segment, &spec, &name, transport) {
        Ok(()) => Ok(Box::new(DoraMemoryPool {
            segment,
            id: spec.id,
        })),
        Err(err) => {
            // The daemon has no record of this pool, so nothing will ever
            // unlink the segment: not `free_memory_pool`, which goes through
            // the daemon's table, and not the daemon's sweeps, which walk it.
            // The name is created `O_EXCL`, so leaking it would also make this
            // pool id permanently unusable. Unmap first, then unlink, so no
            // mapping outlives the name it was made from.
            drop(segment);
            let cleanup = cleanup_clause(naming::unlink_segment(&name), UNLINK_SUPPORTED);
            // Flattened with `{:#}` because cxx converts an error to the C++
            // exception message through `Display`, which prints only the
            // outermost line of a `Report` chain.
            Err(eyre!(
                "failed to register memory pool `{}` with the daemon: {err:#}{cleanup}",
                spec.id
            ))
        }
    }
}

#[allow(clippy::borrowed_box)] // signature dictated by cxx::bridge
fn pool_shm_base(pool: &Box<DoraMemoryPool>) -> u64 {
    pool.segment.shm_base()
}

#[allow(clippy::borrowed_box)] // signature dictated by cxx::bridge
fn pool_segment_bytes(pool: &Box<DoraMemoryPool>) -> usize {
    pool.segment.segment_bytes()
}

#[allow(clippy::borrowed_box)] // signature dictated by cxx::bridge
fn pool_payload(pool: &Box<DoraMemoryPool>, out_ptr: &mut u64, out_len: &mut usize) -> bool {
    match pool.segment.payload() {
        Some((ptr, len)) => {
            *out_ptr = ptr;
            *out_len = len;
            true
        }
        None => false,
    }
}

#[allow(clippy::borrowed_box)] // signature dictated by cxx::bridge
fn pool_payload_offset(pool: &Box<DoraMemoryPool>, out_offset: &mut usize) -> bool {
    match pool.segment.payload_offset() {
        Some(offset) => {
            *out_offset = offset;
            true
        }
        None => false,
    }
}

#[allow(clippy::borrowed_box)] // signature dictated by cxx::bridge
fn pool_payload_len(pool: &Box<DoraMemoryPool>) -> usize {
    pool.segment.payload_len()
}

#[allow(clippy::borrowed_box)] // signature dictated by cxx::bridge
fn pool_id(pool: &Box<DoraMemoryPool>) -> String {
    pool.id.clone()
}

#[allow(clippy::borrowed_box)] // signature dictated by cxx::bridge
fn pool_shm_name(pool: &Box<DoraMemoryPool>) -> String {
    pool.segment.name().to_string()
}

#[allow(clippy::borrowed_box)] // signature dictated by cxx::bridge
fn pool_transport(pool: &Box<DoraMemoryPool>) -> String {
    pool.segment.transport().as_str().to_string()
}

#[allow(clippy::borrowed_box)] // signature dictated by cxx::bridge
fn pool_dtype(pool: &Box<DoraMemoryPool>) -> String {
    pool.segment.dtype().to_string()
}

#[allow(clippy::borrowed_box)] // signature dictated by cxx::bridge
fn pool_shape(pool: &Box<DoraMemoryPool>) -> Vec<usize> {
    pool.segment.shape().to_vec()
}

#[allow(clippy::borrowed_box)] // signature dictated by cxx::bridge
fn pool_ipc_present(pool: &Box<DoraMemoryPool>) -> bool {
    pool.segment.ipc_present()
}

fn pool_begin_write(pool: &mut Box<DoraMemoryPool>) -> ffi::DoraResult {
    match pool.segment.begin_write() {
        Ok(()) => ffi::DoraResult {
            error: String::new(),
        },
        Err(error) => ffi::DoraResult { error },
    }
}

fn pool_end_write(pool: &mut Box<DoraMemoryPool>, ok: bool) {
    pool.segment.end_write(ok)
}

#[allow(clippy::borrowed_box)] // signature dictated by cxx::bridge
fn pool_write_in_progress(pool: &Box<DoraMemoryPool>) -> bool {
    pool.segment.write_in_progress()
}

fn write_memory_pool(pool: &mut Box<DoraMemoryPool>, data: &[u8]) -> ffi::DoraResult {
    match pool.segment.write(data) {
        Ok(()) => ffi::DoraResult {
            error: String::new(),
        },
        Err(error) => ffi::DoraResult { error },
    }
}

#[allow(clippy::boxed_local)]
fn free_memory_pool(
    output_sender: &mut Box<OutputSender>,
    pool: Box<DoraMemoryPool>,
) -> ffi::DoraResult {
    // Daemon only, deliberately. `free_pinned_memory` unlinks the segment and
    // notifies every node that touched the pool; unlinking here as well would
    // bypass that table, and — when the daemon call fails — would remove a
    // segment the daemon still believes exists and will hand to a consumer.
    // The mapping is released when `pool` drops at the end of this call.
    let error = match output_sender.0.free_pinned_memory(pool.id.clone()) {
        Ok(()) => String::new(),
        Err(err) => format!("failed to free memory pool `{}`: {err:#}", pool.id),
    };
    ffi::DoraResult { error }
}

// ---------------------------------------------------------------------
// Memory-pool transport (consumer side)
// ---------------------------------------------------------------------

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering as AtomicOrdering};
use std::sync::{Arc, LazyLock, Mutex};

use dora_memory_pool::segment::OpeningSample;

/// Liveness flags of every open view, keyed by pool id.
///
/// When any node frees a pool the daemon notifies this one, and the flags here
/// flip so that no accessor hands out a pointer into a pool the daemon has
/// released. The mapping itself stays valid — this process still holds it, and
/// `unlink` does not unmap — so a read racing the notification is not a
/// use-after-free; what the flag prevents is a *new* pointer being taken after
/// the pool is gone, and a CUDA consumer being handed a base it can no longer
/// tell apart from a live one.
static VIEW_REGISTRY: LazyLock<Mutex<HashMap<String, Vec<Arc<AtomicBool>>>>> =
    LazyLock::new(|| Mutex::new(HashMap::new()));

/// Distinguishes views from one another, so a read token cannot be applied to
/// the view that did not issue it. A counter rather than the segment address:
/// two views of one segment must still be distinguishable, and an address is
/// reused once a mapping is dropped.
static NEXT_VIEW_SERIAL: AtomicU64 = AtomicU64::new(1);

fn view_registry() -> std::sync::MutexGuard<'static, HashMap<String, Vec<Arc<AtomicBool>>>> {
    VIEW_REGISTRY
        .lock()
        .unwrap_or_else(|poison| poison.into_inner())
}

/// A read-only mapping of a pool, which may belong to another node or another
/// language binding. Dropping it unmaps; releasing the pool is the daemon's
/// job, at the request of whichever node owns it.
pub struct DoraMemoryPoolView {
    segment: PoolSegment,
    id: String,
    serial: u64,
    alive: Arc<AtomicBool>,
}

/// The opening generation sample of a zero-copy read, plus the serial of the
/// view that took it.
///
/// Opaque to C++, which is the point: `OpeningSample` is deliberately not a
/// number even in Rust, and flattening it to a `uint64_t` here would hand the
/// caller something it could forge, compare with the wrong fence, or hold
/// across frames — exactly what the type exists to prevent.
pub struct DoraPoolRead {
    view_serial: u64,
    sample: OpeningSample,
}

/// Register a mapped segment as a live view. Shared with the tests, which have
/// no daemon to ask for a pool.
fn register_view(segment: PoolSegment, id: String) -> DoraMemoryPoolView {
    let alive = Arc::new(AtomicBool::new(true));
    view_registry()
        .entry(id.clone())
        .or_default()
        .push(alive.clone());
    DoraMemoryPoolView {
        segment,
        id,
        serial: NEXT_VIEW_SERIAL.fetch_add(1, AtomicOrdering::Relaxed),
        alive,
    }
}

/// Mark every view of every named pool dead, forget them, and hand the ids
/// back for the caller to act on.
///
/// Consuming and returning the ids is the point rather than a convenience: it
/// is the only way to obtain them, so a caller cannot pass them to C++ without
/// having killed the views first. It also puts the rule somewhere a test can
/// reach — nothing in this crate can push a notification into the event
/// stream's pending set, which only the daemon-facing event thread fills.
fn mark_freed(pool_ids: Vec<String>) -> Vec<String> {
    if !pool_ids.is_empty() {
        let mut registry = view_registry();
        for id in &pool_ids {
            for flag in registry.remove(id).unwrap_or_default() {
                flag.store(false, AtomicOrdering::Release);
            }
        }
    }
    pool_ids
}

impl DoraMemoryPoolView {
    fn is_alive(&self) -> bool {
        self.alive.load(AtomicOrdering::Acquire)
    }

    /// The payload pair, or `None` when there is none to hand out: an `ipc`
    /// pool, whose bytes are in device memory, or a pool that has been freed.
    /// Every byte-reaching accessor goes through this, so neither case can be
    /// missed in one of them.
    fn live_payload(&self) -> Option<(u64, usize)> {
        if self.is_alive() {
            self.segment.payload()
        } else {
            None
        }
    }
}

impl Drop for DoraMemoryPoolView {
    fn drop(&mut self) {
        let mut registry = view_registry();
        if let Some(flags) = registry.get_mut(&self.id) {
            flags.retain(|flag| !Arc::ptr_eq(flag, &self.alive));
            if flags.is_empty() {
                registry.remove(&self.id);
            }
        }
    }
}

/// The segment name for a pool, from the daemon's reply.
///
/// Never derived from the pool id: the name embeds the dataflow and the *owning
/// node's* id, neither of which a consumer knows, and a guessed name would map
/// whatever segment happened to answer to it rather than the pool the daemon
/// has a record of.
fn pool_segment_name(metadata: &DoraMetadata, pool_id: &str) -> EyreResult<String> {
    match metadata.parameters.get("shared_memory_name") {
        Some(DoraParameter::String(name)) if !name.is_empty() => Ok(name.clone()),
        Some(DoraParameter::String(_)) | None => bail!(
            "memory pool `{pool_id}` has no `shared_memory_name`, so it cannot be mapped; a \
             pool that was registered without a segment, or one that has just been freed, \
             looks like this"
        ),
        Some(other) => bail!(
            "memory pool `{pool_id}` reported a `shared_memory_name` that is not a string \
             ({other:?})"
        ),
    }
}

fn read_memory_pool(
    output_sender: &mut Box<OutputSender>,
    pool_id: &str,
) -> EyreResult<Box<DoraMemoryPoolView>> {
    // `free: false` — reading a pool must not release it out from under the
    // node that owns it, and the daemon strips `shared_memory_name` from a
    // freeing read anyway, which would make the reply unmappable.
    let metadata = output_sender
        .0
        .read_pinned_memory(pool_id.to_string(), false)
        // Flattened with `{:#}` because cxx converts an error to the C++
        // exception message through `Display`, which prints only the outermost
        // line of a `Report` chain — and the daemon's reason is the inner one.
        .map_err(|err| eyre!("failed to look up memory pool `{pool_id}`: {err:#}"))?;

    let name = pool_segment_name(&metadata, pool_id)?;
    let segment = PoolSegment::open(&name).map_err(|err| eyre!("{err}"))?;

    Ok(Box::new(register_view(segment, pool_id.to_string())))
}

#[allow(clippy::borrowed_box)] // signature dictated by cxx::bridge
fn view_is_alive(view: &Box<DoraMemoryPoolView>) -> bool {
    view.is_alive()
}

#[allow(clippy::borrowed_box)] // signature dictated by cxx::bridge
fn view_mapping(view: &Box<DoraMemoryPoolView>, out_base: &mut u64, out_bytes: &mut usize) -> bool {
    if !view.is_alive() {
        return false;
    }
    *out_base = view.segment.shm_base();
    *out_bytes = view.segment.segment_bytes();
    true
}

#[allow(clippy::borrowed_box)] // signature dictated by cxx::bridge
fn view_payload(view: &Box<DoraMemoryPoolView>, out_ptr: &mut u64, out_len: &mut usize) -> bool {
    match view.live_payload() {
        Some((ptr, len)) => {
            *out_ptr = ptr;
            *out_len = len;
            true
        }
        None => false,
    }
}

#[allow(clippy::borrowed_box)] // signature dictated by cxx::bridge
fn view_payload_offset(view: &Box<DoraMemoryPoolView>, out_offset: &mut usize) -> bool {
    match view
        .is_alive()
        .then(|| view.segment.payload_offset())
        .flatten()
    {
        Some(offset) => {
            *out_offset = offset;
            true
        }
        None => false,
    }
}

#[allow(clippy::borrowed_box)] // signature dictated by cxx::bridge
fn view_payload_len(view: &Box<DoraMemoryPoolView>) -> usize {
    view.live_payload().map_or(0, |(_, len)| len)
}

#[allow(clippy::borrowed_box)] // signature dictated by cxx::bridge
fn view_id(view: &Box<DoraMemoryPoolView>) -> String {
    view.id.clone()
}

#[allow(clippy::borrowed_box)] // signature dictated by cxx::bridge
fn view_shm_name(view: &Box<DoraMemoryPoolView>) -> String {
    view.segment.name().to_string()
}

#[allow(clippy::borrowed_box)] // signature dictated by cxx::bridge
fn view_transport(view: &Box<DoraMemoryPoolView>) -> String {
    view.segment.transport().as_str().to_string()
}

#[allow(clippy::borrowed_box)] // signature dictated by cxx::bridge
fn view_dtype(view: &Box<DoraMemoryPoolView>) -> String {
    view.segment.dtype().to_string()
}

#[allow(clippy::borrowed_box)] // signature dictated by cxx::bridge
fn view_shape(view: &Box<DoraMemoryPoolView>) -> Vec<usize> {
    view.segment.shape().to_vec()
}

#[allow(clippy::borrowed_box)] // signature dictated by cxx::bridge
fn view_ipc_present(view: &Box<DoraMemoryPoolView>) -> bool {
    view.segment.ipc_present()
}

#[allow(clippy::borrowed_box)] // signature dictated by cxx::bridge
fn view_ipc_handle(view: &Box<DoraMemoryPoolView>) -> Vec<u8> {
    if !view.is_alive() {
        return Vec::new();
    }
    view.segment
        .ipc_handle()
        .map(|handle| handle.to_vec())
        .unwrap_or_default()
}

#[allow(clippy::borrowed_box)] // signature dictated by cxx::bridge
fn view_try_read(view: &Box<DoraMemoryPoolView>, dst: &mut [u8]) -> bool {
    let Some((ptr, len)) = view.live_payload() else {
        return false;
    };
    // Exact, for the reason `write_memory_pool` is exact: a pool frame is
    // fixed-size and nothing on the wire records a per-frame length, so a
    // short destination would silently deliver a prefix as a whole frame.
    if dst.len() != len {
        return false;
    }

    let opening = view.segment.begin_read();
    if !opening.is_complete() {
        // A writer is mid-frame. Skipping the copy is not just an
        // optimisation: it keeps the racing read off the payload entirely on
        // the one path where the result is guaranteed to be discarded.
        return false;
    }
    // Safe against the mapping disappearing — this view owns it — but
    // deliberately racy against the writer's stores, which is what the
    // seqlock bracket around it exists to detect. See `PoolSegment`'s note.
    unsafe {
        std::ptr::copy_nonoverlapping(ptr as *const u8, dst.as_mut_ptr(), len);
    }
    view.segment.read_valid(opening)
}

#[allow(clippy::borrowed_box)] // signature dictated by cxx::bridge
fn view_begin_read(view: &Box<DoraMemoryPoolView>) -> Box<DoraPoolRead> {
    Box::new(DoraPoolRead {
        view_serial: view.serial,
        sample: view.segment.begin_read(),
    })
}

#[allow(clippy::borrowed_box)] // signature dictated by cxx::bridge
fn view_read_valid(view: &Box<DoraMemoryPoolView>, read: &Box<DoraPoolRead>) -> bool {
    view.is_alive() && read.view_serial == view.serial && view.segment.read_valid(read.sample)
}

fn take_freed_pools() -> Vec<String> {
    // `event_stream` and `event_stream::memory_pool` are both `pub mod`, so
    // this path needs no re-export. The set is filled by the event thread when
    // the daemon sends `FreeMemoryPool`, and draining it is destructive — a
    // caller that drops the returned ids loses the notification.
    // `mark_freed` kills the views before handing the ids back, so a node
    // acting on them is already looking at accessors that refuse to produce an
    // address. Routing the ids *through* it, rather than marking and returning
    // separately, is what makes that ordering unskippable here.
    mark_freed(dora_node_api::event_stream::memory_pool::drain_freed_pools())
}

// ---------------------------------------------------------------------
// Service (request/reply) and Action (goal/feedback/result)
// ---------------------------------------------------------------------

fn new_request_id() -> String {
    dora_node_api::DoraNode::new_request_id()
}

fn new_goal_id() -> String {
    dora_node_api::DoraNode::new_goal_id()
}

fn goal_status_succeeded() -> String {
    dora_node_api::GOAL_STATUS_SUCCEEDED.to_string()
}

fn goal_status_aborted() -> String {
    dora_node_api::GOAL_STATUS_ABORTED.to_string()
}

fn goal_status_canceled() -> String {
    dora_node_api::GOAL_STATUS_CANCELED.to_string()
}

#[allow(clippy::boxed_local)] // `Box<Metadata>` is mandated by the cxx bridge signature.
fn send_service_request(
    sender: &mut Box<OutputSender>,
    output_id: String,
    data: &[u8],
    metadata: Box<Metadata>,
) -> ffi::DoraRequestId {
    let mut parameters = (*metadata).into_parameters();
    let request_id = insert_request_id(&mut parameters);

    let result = send_output_internal(sender, output_id, data, parameters);
    finish_request(request_id, result)
}

unsafe fn send_arrow_service_request(
    sender: &mut Box<OutputSender>,
    output_id: String,
    array_ptr: *mut u8,
    schema_ptr: *mut u8,
    mut metadata: Box<Metadata>,
) -> ffi::DoraRequestId {
    let request_id = insert_request_id(&mut metadata.parameters);

    let result =
        unsafe { send_arrow_output_impl(sender, output_id, array_ptr, schema_ptr, Some(metadata)) };
    finish_request(request_id, result)
}

/// Overwrite `parameters`'s `request_id` with a fresh UUID v7 and return
/// it, matching `DoraNode::send_service_request`. Callers must not rely
/// on a self-supplied id surviving: the whole point of the helper is
/// that the framework owns the correlation key.
fn insert_request_id(parameters: &mut DoraMetadataParameters) -> String {
    let request_id = dora_node_api::DoraNode::new_request_id();
    parameters.insert(
        dora_node_api::REQUEST_ID.to_string(),
        DoraParameter::String(request_id.clone()),
    );
    request_id
}

/// Pair the generated request id with the send outcome. The id is only
/// meaningful if the send succeeded, so an error clears it — otherwise a
/// caller that skipped the `error` check would wait on a correlation that
/// was never put on the wire.
fn finish_request(request_id: String, result: ffi::DoraResult) -> ffi::DoraRequestId {
    if result.error.is_empty() {
        ffi::DoraRequestId {
            request_id,
            error: String::new(),
        }
    } else {
        ffi::DoraRequestId {
            request_id: String::new(),
            error: result.error,
        }
    }
}

/// Semantic alias of `send_output_with_metadata`, mirroring
/// `DoraNode::send_service_response`. It exists so the server side of an
/// exchange reads as a reply rather than an unrelated output; the
/// correlation itself travels in the passed-through `metadata`.
fn send_service_response(
    sender: &mut Box<OutputSender>,
    output_id: String,
    data: &[u8],
    metadata: Box<Metadata>,
) -> ffi::DoraResult {
    send_output_with_metadata(sender, output_id, data, metadata)
}

fn recv_service_response(
    events: &mut Box<Events>,
    request_id: &str,
    server_node_id: &str,
    timeout_ms: u64,
) -> ffi::DoraPatternResult {
    let (server, timeout) = match pattern_wait_args(server_node_id, timeout_ms) {
        Ok(args) => args,
        Err(result) => return result,
    };
    pattern_result(futures_lite::future::block_on(
        events.0.recv_service_response(request_id, &server, timeout),
    ))
}

fn recv_action_result(
    events: &mut Box<Events>,
    goal_id: &str,
    server_node_id: &str,
    timeout_ms: u64,
) -> ffi::DoraPatternResult {
    let (server, timeout) = match pattern_wait_args(server_node_id, timeout_ms) {
        Ok(args) => args,
        Err(result) => return result,
    };
    pattern_result(futures_lite::future::block_on(
        events.0.recv_action_result(goal_id, &server, timeout),
    ))
}

/// Validate and normalise the arguments shared by both pattern-aware
/// waits, before anything is awaited.
///
/// The node id is parsed via `FromStr` rather than `From<String>`: the
/// latter is documented as panicking on invalid characters
/// (`libraries/message/src/id.rs`), so a typo in a C++ string literal
/// would abort the whole node instead of returning an error.
fn pattern_wait_args(
    server_node_id: &str,
    timeout_ms: u64,
) -> Result<(NodeId, Duration), ffi::DoraPatternResult> {
    let server = server_node_id.parse::<NodeId>().map_err(|e| {
        pattern_failure(
            ffi::DoraPatternStatus::InvalidArgument,
            format!("invalid server node id '{server_node_id}': {e}"),
            EventOrReason::Empty,
        )
    })?;
    Ok((server, clamp_pattern_timeout(timeout_ms)))
}

/// Convert `timeout_ms` into a `Duration` that `Instant::now() + dur`
/// can represent.
///
/// `timeout_ms` crosses the bridge as an unbounded `u64`, and the
/// pattern helpers compute `Instant::now() + timeout` internally, which
/// panics on overflow. Halving until the sum is representable keeps a
/// bogus `UINT64_MAX` from a C++ caller behaving like "wait a very long
/// time" instead of aborting the process — the same defensive stance
/// `next_event_timeout` takes.
fn clamp_pattern_timeout(timeout_ms: u64) -> Duration {
    let mut timeout = Duration::from_millis(timeout_ms);
    let now = Instant::now();
    while !timeout.is_zero() && now.checked_add(timeout).is_none() {
        timeout /= 2;
    }
    timeout
}

fn pattern_result(outcome: Result<Event, PatternError>) -> ffi::DoraPatternResult {
    match outcome {
        Ok(event) => ffi::DoraPatternResult {
            status: ffi::DoraPatternStatus::Matched,
            error: String::new(),
            event: Box::new(DoraEvent(EventOrReason::Event(event))),
        },
        // Each failure also gets an `EventOrReason` whose `event_type`
        // matches the status, so C++ code that only inspects the event
        // (the pre-existing idiom) still sees something sensible.
        Err(err @ PatternError::Timeout) => pattern_failure(
            ffi::DoraPatternStatus::Timeout,
            err.to_string(),
            EventOrReason::TimedOut,
        ),
        Err(err @ PatternError::ServerRestarted(_)) => pattern_failure(
            ffi::DoraPatternStatus::ServerRestarted,
            err.to_string(),
            EventOrReason::Empty,
        ),
        Err(err @ PatternError::StreamEnded) => pattern_failure(
            ffi::DoraPatternStatus::StreamEnded,
            err.to_string(),
            EventOrReason::Closed,
        ),
        Err(err @ PatternError::StreamError(_)) => pattern_failure(
            ffi::DoraPatternStatus::StreamError,
            err.to_string(),
            EventOrReason::Empty,
        ),
    }
}

fn pattern_failure(
    status: ffi::DoraPatternStatus,
    error: String,
    event: EventOrReason,
) -> ffi::DoraPatternResult {
    ffi::DoraPatternResult {
        status,
        error,
        event: Box::new(DoraEvent(event)),
    }
}

pub struct MergedEvents {
    events: Option<Box<dyn Stream<Item = MergedEvent<ExternalEvent>> + Unpin>>,
    #[cfg(any(feature = "ros2-bridge", test))]
    next_id: u32,
}

fn new_metadata() -> Box<Metadata> {
    Box::new(Metadata::empty())
}
unsafe fn send_arrow_output(
    sender: &mut Box<OutputSender>,
    id: String,
    array_ptr: *mut u8,
    schema_ptr: *mut u8,
) -> ffi::DoraResult {
    unsafe { send_arrow_output_impl(sender, id, array_ptr, schema_ptr, None) }
}

unsafe fn send_arrow_output_with_metadata(
    sender: &mut Box<OutputSender>,
    id: String,
    array_ptr: *mut u8,
    schema_ptr: *mut u8,
    metadata: Box<Metadata>,
) -> ffi::DoraResult {
    unsafe { send_arrow_output_impl(sender, id, array_ptr, schema_ptr, Some(metadata)) }
}

unsafe fn send_arrow_output_impl(
    sender: &mut Box<OutputSender>,
    id: String,
    array_ptr: *mut u8,
    schema_ptr: *mut u8,
    metadata: Option<Box<Metadata>>,
) -> ffi::DoraResult {
    let array_ptr = array_ptr as *mut arrow::ffi::FFI_ArrowArray;
    let schema_ptr = schema_ptr as *mut arrow::ffi::FFI_ArrowSchema;

    if array_ptr.is_null() || schema_ptr.is_null() {
        return ffi::DoraResult {
            error: "Received null Arrow array or schema pointer".to_string(),
        };
    }

    let array = unsafe { std::ptr::read(array_ptr) };
    let schema = unsafe { std::ptr::read(schema_ptr) };

    unsafe {
        std::ptr::write(array_ptr, std::mem::zeroed());
        std::ptr::write(schema_ptr, std::mem::zeroed());
    }

    match unsafe { arrow::ffi::from_ffi(array, &schema) } {
        Ok(array_data) => {
            let arrow_array = arrow::array::make_array(array_data);
            let parameters: DoraMetadataParameters = metadata
                .as_ref()
                .map(|metadata| metadata.parameters.clone())
                .unwrap_or_default();
            let output_id = match parse_output_id(&id) {
                Ok(parsed) => parsed,
                Err(err) => return err,
            };
            let result = sender.0.send_output(output_id, parameters, arrow_array);
            match result {
                Ok(()) => ffi::DoraResult {
                    error: String::new(),
                },
                Err(err) => ffi::DoraResult {
                    error: format!("{err:?}"),
                },
            }
        }
        Err(e) => ffi::DoraResult {
            error: format!("Error importing array from C++: {e:?}"),
        },
    }
}

impl MergedEvents {
    #[cfg(any(feature = "ros2-bridge", test))]
    fn merge(&mut self, events: impl Stream<Item = Box<dyn Any>> + Unpin + 'static) -> u32 {
        let id = self.next_id;
        self.next_id += 1;
        let events = Box::pin(events.map(move |event| ExternalEvent { event, id }));

        let inner = self.events.take().unwrap();
        let merged: Box<dyn Stream<Item = _> + Unpin + 'static> =
            Box::new(inner.merge_external(events).map(|event| match event {
                MergedEvent::Dora(event) => MergedEvent::Dora(event),
                MergedEvent::External(event) => MergedEvent::External(event.flatten()),
            }));
        self.events = Some(merged);

        id
    }

    fn next(&mut self) -> MergedDoraEvent {
        let event = futures_lite::future::block_on(self.events.as_mut().unwrap().next());
        MergedDoraEvent(event)
    }
}

impl ffi::CombinedEvents {
    fn next(&mut self) -> ffi::CombinedEvent {
        ffi::CombinedEvent {
            event: Box::new(self.events.next()),
        }
    }
}

pub struct MergedDoraEvent(Option<MergedEvent<ExternalEvent>>);

pub struct ExternalEvent {
    pub event: Box<dyn Any>,
    pub id: u32,
}

impl ffi::CombinedEvent {
    fn is_dora(&self) -> bool {
        matches!(&self.event.0, Some(MergedEvent::Dora(_)))
    }
}

fn downcast_dora(event: ffi::CombinedEvent) -> eyre::Result<Box<DoraEvent>> {
    match event.event.0 {
        Some(MergedEvent::Dora(event)) => Ok(Box::new(DoraEvent(EventOrReason::Event(event)))),
        // None means the merged stream closed; pass that through as Closed
        // so C++ sees `DoraEventType::AllInputsClosed` rather than a
        // bail-out error.
        None => Ok(Box::new(DoraEvent(EventOrReason::Closed))),
        _ => eyre::bail!("not a dora event"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use dora_node_api::{
        ArrowData,
        arrow::array::{ArrayRef, Int32Array},
        uhlc::HLC,
    };
    use std::sync::Arc;

    /// Regression test for #2030: a non-UInt8 input (e.g. Int32 from another
    /// node) must return an `Err` from `event_as_input` instead of aborting
    /// the process via `todo!()`.
    #[test]
    fn event_as_input_non_uint8_returns_err() {
        let array: ArrayRef = Arc::new(Int32Array::from(vec![1, 2, 3]));
        let event = Box::new(DoraEvent(EventOrReason::Event(Event::Input {
            id: "my_input".into(),
            metadata: DoraMetadata::new(HLC::default().new_timestamp()),
            data: ArrowData(array),
        })));

        let result = event_as_input(event);
        assert!(result.is_err(), "expected Err for non-UInt8 input, got Ok");
    }

    #[test]
    fn merged_events_assigns_ids_to_external_streams() {
        let mut events = MergedEvents {
            events: Some(Box::new(stream::empty())),
            #[cfg(any(feature = "ros2-bridge", test))]
            next_id: 1,
        };

        let first_id = events.merge(stream::once(Box::new("first") as Box<dyn Any>));
        let second_id = events.merge(stream::once(Box::new("second") as Box<dyn Any>));

        assert_eq!(first_id, 1);
        assert_eq!(second_id, 2);

        let mut seen = Vec::new();
        for _ in 0..2 {
            if let Some(MergedEvent::External(event)) = events.next().0 {
                seen.push(event.id);
            }
        }
        seen.sort_unstable();
        assert_eq!(seen, vec![1, 2]);
    }

    /// Regression test: a caller-supplied output id with invalid characters
    /// (e.g. a typo containing a space) must surface as a `DoraResult.error`
    /// rather than panicking across the `cxx::bridge` and aborting the node.
    /// The `send_output`/`send_arrow_output`/`close_outputs` paths all route
    /// through `parse_output_id`.
    #[test]
    fn parse_output_id_rejects_invalid_without_panicking() {
        let ok = parse_output_id("cmd_vel");
        assert!(ok.is_ok(), "a valid id must parse");

        for bad in ["cmd vel", "cmd!", "señal"] {
            let err = parse_output_id(bad).expect_err("invalid id must return an error");
            assert!(
                err.error.contains("invalid output id"),
                "error must describe the failure, got {:?}",
                err.error
            );
            assert!(!err.error.is_empty());
        }
    }

    // ---- #2686: Service / Action parity for the C++ binding ----

    /// The server side of a Service exchange reads `request_id` off the
    /// incoming request through this accessor, so it must surface both
    /// the payload bytes and the metadata.
    #[test]
    fn event_as_input_with_metadata_exposes_payload_and_correlation() {
        let mut metadata = DoraMetadata::new(HLC::default().new_timestamp());
        metadata.parameters.insert(
            dora_node_api::REQUEST_ID.to_string(),
            DoraParameter::String("req-1".into()),
        );
        let array: ArrayRef = Arc::new(UInt8Array::from(vec![7u8, 9u8]));

        let input =
            event_as_input_with_metadata(Box::new(DoraEvent(EventOrReason::Event(Event::Input {
                id: "request".into(),
                metadata,
                data: ArrowData(array),
            }))))
            .expect("input event should decode");

        assert_eq!(input.id, "request");
        assert_eq!(input.data, vec![7u8, 9u8]);
        assert_eq!(input.metadata.request_id().unwrap(), "req-1");
    }

    /// It must reject non-input events and non-byte payloads the same
    /// way `event_as_input` does, rather than aborting (cf. #2030).
    #[test]
    fn event_as_input_with_metadata_rejects_bad_input() {
        let not_input = event_as_input_with_metadata(Box::new(DoraEvent(EventOrReason::Empty)));
        assert!(not_input.is_err(), "a non-input event must return Err");

        let array: ArrayRef = Arc::new(Int32Array::from(vec![1, 2, 3]));
        let wrong_type =
            event_as_input_with_metadata(Box::new(DoraEvent(EventOrReason::Event(Event::Input {
                id: "input".into(),
                metadata: DoraMetadata::new(HLC::default().new_timestamp()),
                data: ArrowData(array),
            }))));
        assert!(wrong_type.is_err(), "a non-UInt8 payload must return Err");
    }

    /// The typed correlation setters must write the *exact* reserved keys
    /// the framework matches on. If these drift from
    /// `dora_message::metadata`, a C++ node's requests would look
    /// well-formed but never correlate.
    #[test]
    fn correlation_setters_use_the_reserved_keys() {
        let mut meta = Metadata::empty();
        meta.set_request_id("req-1".into()).unwrap();
        meta.set_goal_id("goal-1".into()).unwrap();
        meta.set_goal_status("succeeded".into()).unwrap();

        assert_eq!(meta.get_str(dora_node_api::REQUEST_ID).unwrap(), "req-1");
        assert_eq!(meta.get_str(dora_node_api::GOAL_ID).unwrap(), "goal-1");
        assert_eq!(
            meta.get_str(dora_node_api::GOAL_STATUS).unwrap(),
            "succeeded"
        );

        assert_eq!(meta.request_id().unwrap(), "req-1");
        assert_eq!(meta.goal_id().unwrap(), "goal-1");
        assert_eq!(meta.goal_status().unwrap(), "succeeded");
    }

    #[test]
    fn correlation_getters_error_when_unset() {
        let meta = Metadata::empty();
        assert!(meta.request_id().is_err());
        assert!(meta.goal_id().is_err());
        assert!(meta.goal_status().is_err());
    }

    /// The exported terminal statuses must equal the framework
    /// constants; `recv_action_result` only treats these three as
    /// terminal, so a typo here would make waits hang until timeout.
    #[test]
    fn exported_goal_statuses_match_framework_constants() {
        assert_eq!(
            goal_status_succeeded(),
            dora_node_api::GOAL_STATUS_SUCCEEDED
        );
        assert_eq!(goal_status_aborted(), dora_node_api::GOAL_STATUS_ABORTED);
        assert_eq!(goal_status_canceled(), dora_node_api::GOAL_STATUS_CANCELED);
    }

    #[test]
    fn new_request_id_is_a_unique_uuid() {
        let first = new_request_id();
        let second = new_request_id();
        dora_node_api::uuid::Uuid::parse_str(&first).expect("request id should be a valid UUID");
        assert_ne!(first, second, "successive request ids must differ");
        // `new_goal_id` is documented as an alias, so it must draw from
        // the same generator rather than being a distinct id space.
        dora_node_api::uuid::Uuid::parse_str(&new_goal_id()).expect("goal id should be a UUID");
    }

    /// A caller-supplied `request_id` must be replaced, matching
    /// `DoraNode::send_service_request`. Silently honouring it would let
    /// two in-flight requests share a correlation key.
    #[test]
    fn insert_request_id_overwrites_caller_value() {
        let mut parameters = DoraMetadataParameters::default();
        parameters.insert(
            dora_node_api::REQUEST_ID.to_string(),
            DoraParameter::String("caller-supplied".into()),
        );

        let generated = insert_request_id(&mut parameters);

        assert_ne!(generated, "caller-supplied");
        assert_eq!(
            parameters.get(dora_node_api::REQUEST_ID),
            Some(&DoraParameter::String(generated))
        );
    }

    /// A failed send must not hand back a request id: a caller that
    /// skipped the `error` check would otherwise block in
    /// `recv_service_response` on a correlation never put on the wire.
    #[test]
    fn finish_request_clears_id_on_send_failure() {
        let ok = finish_request(
            "req-1".into(),
            ffi::DoraResult {
                error: String::new(),
            },
        );
        assert_eq!(ok.request_id, "req-1");
        assert!(ok.error.is_empty());

        let failed = finish_request(
            "req-1".into(),
            ffi::DoraResult {
                error: "send failed".into(),
            },
        );
        assert!(
            failed.request_id.is_empty(),
            "a failed send must not return a usable request id"
        );
        assert_eq!(failed.error, "send failed");
    }

    /// `timeout_ms` is an unbounded `u64` at the FFI boundary while the
    /// pattern helpers compute `Instant::now() + timeout` internally.
    /// A bogus `UINT64_MAX` from C++ must clamp, not panic.
    #[test]
    fn clamp_pattern_timeout_survives_extreme_values() {
        assert_eq!(clamp_pattern_timeout(0), Duration::ZERO);
        assert_eq!(clamp_pattern_timeout(5_000), Duration::from_millis(5_000));

        let clamped = clamp_pattern_timeout(u64::MAX);
        assert!(
            !clamped.is_zero(),
            "an overflowing timeout must clamp to a long wait, not to zero"
        );
        Instant::now()
            .checked_add(clamped)
            .expect("clamped timeout must be representable as a deadline");
    }

    /// An invalid node id must surface as `InvalidArgument` rather than
    /// panicking across the bridge (`NodeId::from(String)` panics), and
    /// it must be rejected *before* any waiting happens.
    #[test]
    fn invalid_server_node_id_is_reported_not_panicked() {
        let Err(err) = pattern_wait_args("bad id/with slash", 1_000) else {
            panic!("malformed node id must be rejected");
        };

        assert!(matches!(
            err.status,
            ffi::DoraPatternStatus::InvalidArgument
        ));
        assert!(err.error.contains("invalid server node id"));
        assert!(matches!(event_type(&err.event), ffi::DoraEventType::Empty));

        // `DoraPatternResult` (the error type here) is a cxx shared
        // struct and so has no `Debug`; destructure instead of unwrapping.
        let Ok((server, timeout)) = pattern_wait_args("service-server", 1_000) else {
            panic!("a valid node id must parse");
        };
        assert_eq!(server.to_string(), "service-server");
        assert_eq!(timeout, Duration::from_millis(1_000));
    }

    /// Every `PatternError` must map to a distinct status *and* to an
    /// event whose `event_type` tells the same story, so C++ code can
    /// branch on either field.
    #[test]
    fn pattern_errors_map_to_matching_kind_and_event_type() {
        let timeout = pattern_result(Err(PatternError::Timeout));
        assert!(matches!(timeout.status, ffi::DoraPatternStatus::Timeout));
        assert!(matches!(
            event_type(&timeout.event),
            ffi::DoraEventType::Timeout
        ));

        let restarted = pattern_result(Err(PatternError::ServerRestarted("server".into())));
        assert!(matches!(
            restarted.status,
            ffi::DoraPatternStatus::ServerRestarted
        ));
        assert!(
            restarted.error.contains("server"),
            "the restarted server's id must reach the C++ caller: {}",
            restarted.error
        );

        let ended = pattern_result(Err(PatternError::StreamEnded));
        assert!(matches!(ended.status, ffi::DoraPatternStatus::StreamEnded));
        assert!(matches!(
            event_type(&ended.event),
            ffi::DoraEventType::AllInputsClosed
        ));

        let stream_error = pattern_result(Err(PatternError::StreamError("boom".into())));
        assert!(matches!(
            stream_error.status,
            ffi::DoraPatternStatus::StreamError
        ));
        assert!(stream_error.error.contains("boom"));
    }

    /// On success the correlated input must come through unchanged, with
    /// no error set.
    #[test]
    fn pattern_result_passes_the_matched_input_through() {
        let mut metadata = DoraMetadata::new(HLC::default().new_timestamp());
        metadata.parameters.insert(
            dora_node_api::REQUEST_ID.to_string(),
            DoraParameter::String("req-1".into()),
        );
        let array: ArrayRef = Arc::new(Int32Array::from(vec![7]));

        let result = pattern_result(Ok(Event::Input {
            id: "response".into(),
            metadata,
            data: ArrowData(array),
        }));

        assert!(matches!(result.status, ffi::DoraPatternStatus::Matched));
        assert!(result.error.is_empty());
        assert!(matches!(
            event_type(&result.event),
            ffi::DoraEventType::Input
        ));
    }

    /// Regression test: `set_timestamp` must accept and correctly represent
    /// pre-epoch (negative) nanosecond timestamps. The previous truncating
    /// `value % 1_000_000_000` produced a negative remainder that wrapped when
    /// cast to `u32`, making `from_timestamp` reject valid instants. The value
    /// must also round-trip through `get_timestamp`.
    #[test]
    fn set_timestamp_roundtrips_negative_values() {
        for value in [
            -1_i64,
            -500_000_000,
            -1_000_000_000,
            -1_500_000_000,
            -1_000_000_001,
            0,
            1,
            1_500_000_000,
        ] {
            let mut meta = Metadata::empty();
            meta.set_timestamp("ts", value)
                .unwrap_or_else(|e| panic!("set_timestamp({value}) failed: {e}"));
            let got = meta
                .get_timestamp("ts")
                .unwrap_or_else(|e| panic!("get_timestamp after set({value}) failed: {e}"));
            assert_eq!(got, value, "timestamp {value} did not round-trip");
        }
    }

    // -----------------------------------------------------------------
    // Memory-pool transport
    // -----------------------------------------------------------------

    /// `auto` must never resolve to a mode whose registration would fail:
    /// `unified` works everywhere a CUDA receiver exists, `shmem` everywhere
    /// else.
    #[test]
    fn auto_transport_follows_the_receiver() {
        assert_eq!(
            resolve_transport_with("auto", true, None).unwrap(),
            Transport::Unified
        );
        assert_eq!(
            resolve_transport_with("auto", false, None).unwrap(),
            Transport::Shmem
        );
    }

    /// An empty string is what a C++ caller that left the field default sends,
    /// and it must mean `auto` rather than "unknown transport".
    #[test]
    fn an_empty_transport_string_means_auto() {
        assert_eq!(
            resolve_transport_with("", true, None).unwrap(),
            Transport::Unified
        );
        assert_eq!(
            resolve_transport_with("", false, None).unwrap(),
            Transport::Shmem
        );
    }

    #[test]
    fn explicit_transport_wins_over_the_receiver_hint() {
        assert_eq!(
            resolve_transport_with("shmem", true, None).unwrap(),
            Transport::Shmem
        );
        assert_eq!(
            resolve_transport_with("unified", false, None).unwrap(),
            Transport::Unified
        );
    }

    /// The environment variable is an operator's escape hatch for `auto` only.
    #[test]
    fn the_environment_overrides_auto_in_both_directions() {
        assert_eq!(
            resolve_transport_with("auto", true, Some("shmem")).unwrap(),
            Transport::Shmem,
            "the environment must be able to veto the receiver hint"
        );
        assert_eq!(
            resolve_transport_with("auto", false, Some("unified")).unwrap(),
            Transport::Unified
        );
        assert_eq!(
            resolve_transport_with("", true, Some("shmem")).unwrap(),
            Transport::Shmem,
            "an empty request is `auto`, so the environment applies to it too"
        );
    }

    /// An explicit request is the node author's decision about its own buffer
    /// layout; an operator's environment variable must not silently rewrite it.
    #[test]
    fn the_environment_does_not_override_an_explicit_request() {
        assert_eq!(
            resolve_transport_with("unified", false, Some("shmem")).unwrap(),
            Transport::Unified
        );
        assert_eq!(
            resolve_transport_with("shmem", true, Some("unified")).unwrap(),
            Transport::Shmem
        );
    }

    /// A misspelled override must fail loudly. Silently falling back to the
    /// receiver hint would hand back the very transport the operator was
    /// trying to change, with no sign that the variable did nothing.
    #[test]
    fn an_unrecognized_environment_value_is_rejected_by_name() {
        for value in ["shmemm", "", "ipc"] {
            let err = resolve_transport_with("auto", true, Some(value))
                .unwrap_err()
                .to_string();
            assert!(
                err.contains("DORA_MEMORY_POOL_TRANSPORT"),
                "the error must name the variable that has to change: {err}"
            );
        }
    }

    /// `ipc` must fail with a message about IPC specifically: a caller asking
    /// for it has assumptions about device memory that this binding cannot
    /// meet, and needs to be told why, not merely that the word was not
    /// recognized.
    #[test]
    fn ipc_transport_is_rejected_with_an_ipc_specific_message() {
        let err = resolve_transport("ipc", true).unwrap_err().to_string();
        assert!(
            err.contains("cudaIpcGetMemHandle"),
            "the error must explain why ipc cannot work here: {err}"
        );
        assert!(
            !err.contains("unknown transport"),
            "ipc must not fall through to the catch-all arm, whose message \
             also mentions `unified`: {err}"
        );
    }

    #[test]
    fn unknown_transport_is_rejected() {
        let err = resolve_transport("cuda-magic", false)
            .unwrap_err()
            .to_string();
        assert!(err.contains("cuda-magic"), "unexpected error: {err}");
    }

    /// The daemon rejects a registration with an empty `dtype` or an empty
    /// `shape`, and the segment is already created by then. Checking first
    /// turns a round trip that leaves a segment to clean up into a local
    /// error that names the field.
    #[test]
    fn an_empty_dtype_is_rejected_before_anything_is_allocated() {
        let err = validate_daemon_required_fields("", &[4]).unwrap_err();
        assert!(err.to_string().contains("dtype"), "unexpected error: {err}");
    }

    #[test]
    fn an_empty_shape_is_rejected_before_anything_is_allocated() {
        let err = validate_daemon_required_fields("uint8", &[]).unwrap_err();
        assert!(err.to_string().contains("shape"), "unexpected error: {err}");
        assert!(
            !err.to_string().contains("dtype"),
            "the shape error must not be reported as a dtype error: {err}"
        );
    }

    #[test]
    fn a_complete_spec_passes_the_precheck() {
        validate_daemon_required_fields("uint8", &[640, 512, 4]).expect("a complete spec");
    }

    /// Off Linux `unlink_segment` fails for every name with the same
    /// "unavailable on this platform" text, which describes the platform and
    /// not the segment. Appending it would read as a second, independent
    /// failure and push the registration error the caller has to act on behind
    /// noise.
    #[test]
    fn an_inapplicable_unlink_adds_no_clause() {
        let clause = cleanup_clause(
            Err("memory-pool transport is unavailable on this platform".to_string()),
            false,
        );
        assert_eq!(clause, "", "unexpected clause: {clause}");
    }

    /// Where unlinking does apply, a failure means a real segment was left in
    /// `/dev/shm` under a name `O_EXCL` will never hand out again, so it has to
    /// be reported.
    #[test]
    fn a_real_unlink_failure_is_reported_with_its_cause() {
        let clause = cleanup_clause(Err("permission denied".to_string()), true);
        assert!(
            clause.contains("orphaned segment"),
            "a leaked segment must be named as such: {clause}"
        );
        assert!(
            clause.contains("permission denied"),
            "the clause must carry the unlink error, not just announce one: {clause}"
        );
    }

    #[test]
    fn a_successful_unlink_adds_no_clause() {
        assert_eq!(cleanup_clause(Ok(()), true), "");
    }

    /// The one platform where the memory-pool transport works must take the
    /// reporting branch; a `cfg` that silently flipped would make every leaked
    /// segment invisible.
    #[test]
    fn unlinking_is_supported_exactly_on_linux() {
        assert_eq!(UNLINK_SUPPORTED, cfg!(target_os = "linux"));
    }

    // -----------------------------------------------------------------
    // Daemon parameter contract
    // -----------------------------------------------------------------

    /// Every key the daemon's `RegisterPinnedMemory` arm reads, paired with the
    /// accessor it reads that key through. Transcribed from
    /// `pool_metadata_from_params` and the validation immediately after it in
    /// `binaries/daemon/src/lib.rs`.
    ///
    /// That function is private to the `dora-daemon` crate, so this is a
    /// transcription rather than a call. It still catches the failure it is
    /// here for — a typo or a wrong variant on the producer side, which is
    /// otherwise a runtime-only "missing shared_memory_name". It cannot catch
    /// the daemon renaming a key; only a shared function could.
    const DAEMON_POOL_KEYS: &[(&str, &str)] = &[
        ("ptr", "get_int"),
        ("size", "get_int"),
        ("dtype", "get_str"),
        ("shape", "ListInt"),
        ("is_pinned", "get_bool"),
        ("pinned_type", "get_str"),
        ("shared_memory_name", "get_str"),
        ("ipc_present", "get_bool"),
        ("buffer_id", "get_str"),
    ];

    /// Which of the daemon's variant-typed accessors will return a value for
    /// this parameter. Each one returns `None` on a variant mismatch, and the
    /// daemon then substitutes a default — so a wrong variant is exactly as
    /// silent as a misspelled key.
    fn daemon_accessor(parameter: &DoraParameter) -> &'static str {
        match parameter {
            DoraParameter::Integer(_) => "get_int",
            DoraParameter::String(_) => "get_str",
            DoraParameter::Bool(_) => "get_bool",
            DoraParameter::ListInt(_) => "ListInt",
            _ => "unread by the daemon",
        }
    }

    fn sample_pool_params() -> (String, DoraMetadataParameters) {
        let spec = ffi::DoraMemoryPoolSpec {
            id: "eo_frames".to_string(),
            size: 640 * 512 * 4,
            dtype: "uint8".to_string(),
            shape: vec![512, 640, 4],
            transport: "shmem".to_string(),
            receiver_is_cuda: false,
        };
        // The real name builder, so the daemon's prefix and traversal checks
        // are exercised against what registration actually sends.
        let name = naming::segment_name("df-uuid", "eo_media", &spec.id).expect("a valid name");
        let params = pool_daemon_params(0x7f00_0000_1000, &spec, &name, Transport::Shmem);
        (name, params)
    }

    /// The daemon looks every one of these up by string and by variant; a
    /// mismatch in either is silently a default, and the registration comes
    /// back naming whichever check happened to trip first.
    #[test]
    fn the_daemon_parameter_map_has_exactly_the_keys_the_daemon_reads() {
        let (_, params) = sample_pool_params();
        let mut expected: Vec<&str> = DAEMON_POOL_KEYS.iter().map(|(key, _)| *key).collect();
        expected.sort_unstable();
        let mut actual: Vec<&str> = params.keys().map(String::as_str).collect();
        actual.sort_unstable();
        assert_eq!(actual, expected, "daemon parameter key set drifted");

        for (key, accessor) in DAEMON_POOL_KEYS {
            let parameter = params
                .get(*key)
                .unwrap_or_else(|| panic!("`{key}` missing from the daemon parameter map"));
            assert_eq!(
                daemon_accessor(parameter),
                *accessor,
                "`{key}` has the wrong parameter variant; the daemon reads it with `{accessor}`, \
                 which would return None and leave a default in its place"
            );
        }
    }

    /// The daemon rejects the registration outright if any of these fails, and
    /// the segment already exists by then. Values, not just keys: `ptr == 0`
    /// and a name without the `dora_pool_` prefix are both rejections that a
    /// well-formed key set would sail past.
    #[test]
    fn the_daemon_parameter_map_passes_the_daemon_validation_checks() {
        let (name, params) = sample_pool_params();

        let ptr = match params.get("ptr") {
            Some(DoraParameter::Integer(value)) => *value,
            other => panic!("ptr is not an integer: {other:?}"),
        };
        assert_ne!(ptr, 0, "the daemon rejects `ptr == 0`");

        let size = match params.get("size") {
            Some(DoraParameter::Integer(value)) => *value,
            other => panic!("size is not an integer: {other:?}"),
        };
        assert_ne!(size, 0, "the daemon rejects `size == 0`");
        assert!(
            size <= 1024 * 1024 * 1024,
            "the daemon caps a pool at 1 GiB, got {size}"
        );

        match params.get("dtype") {
            Some(DoraParameter::String(dtype)) => {
                assert!(!dtype.is_empty(), "the daemon rejects an empty dtype")
            }
            other => panic!("dtype is not a string: {other:?}"),
        }

        match params.get("shape") {
            Some(DoraParameter::ListInt(shape)) => {
                assert!(!shape.is_empty(), "the daemon rejects an empty shape")
            }
            other => panic!("shape is not a list<int>: {other:?}"),
        }

        // Exactly the daemon's `shared_memory_name` checks, which are also what
        // decides whether it will ever unlink the segment again.
        match params.get("shared_memory_name") {
            Some(DoraParameter::String(shm_name)) => {
                assert_eq!(shm_name, &name);
                assert!(
                    shm_name.starts_with("dora_pool_"),
                    "the daemon only unlinks names under its own prefix: {shm_name}"
                );
                assert!(
                    !shm_name.contains('/'),
                    "rejected by the daemon: {shm_name}"
                );
                assert!(
                    !shm_name.contains(".."),
                    "rejected by the daemon: {shm_name}"
                );
            }
            other => panic!("shared_memory_name is not a string: {other:?}"),
        }
    }

    /// `pinned_type` decides how the daemon describes the pool to a consumer,
    /// and it is the one value here that varies with the resolved transport.
    #[test]
    fn pinned_type_follows_the_resolved_transport() {
        let spec = ffi::DoraMemoryPoolSpec {
            id: "p".to_string(),
            size: 8,
            dtype: "uint8".to_string(),
            shape: vec![8],
            transport: String::new(),
            receiver_is_cuda: false,
        };
        for (transport, expected) in [(Transport::Shmem, "cpu"), (Transport::Unified, "cuda")] {
            let params = pool_daemon_params(0x1000, &spec, "dora_pool_a_b_p", transport);
            match params.get("pinned_type") {
                Some(DoraParameter::String(value)) => assert_eq!(value, expected, "{transport:?}"),
                other => panic!("pinned_type is not a string: {other:?}"),
            }
        }
    }

    // -----------------------------------------------------------------
    // Consumer side: segment name resolution
    // -----------------------------------------------------------------

    fn metadata_with(parameter: Option<DoraParameter>) -> DoraMetadata {
        let mut parameters = DoraMetadataParameters::new();
        if let Some(parameter) = parameter {
            parameters.insert("shared_memory_name".to_string(), parameter);
        }
        DoraMetadata::from_parameters(HLC::default().new_timestamp(), parameters)
    }

    #[test]
    fn the_segment_name_comes_from_the_daemons_reply() {
        let metadata = metadata_with(Some(DoraParameter::String("dora_pool_df_n_p".to_string())));
        assert_eq!(
            pool_segment_name(&metadata, "p").expect("a named pool maps"),
            "dora_pool_df_n_p"
        );
    }

    /// The daemon strips `shared_memory_name` from the reply to a freeing read,
    /// and a pool registered without a segment never had one. Guessing a name
    /// from the pool id would map whatever segment answered to it — the id does
    /// not determine the name, which embeds the dataflow and the *owning*
    /// node's id.
    #[test]
    fn a_reply_without_a_usable_segment_name_is_refused_rather_than_guessed() {
        for parameter in [
            None,
            Some(DoraParameter::String(String::new())),
            Some(DoraParameter::Integer(7)),
        ] {
            let metadata = metadata_with(parameter.clone());
            let err = pool_segment_name(&metadata, "eo_frames")
                .expect_err("an unusable name must not be mapped");
            let message = format!("{err:#}");
            assert!(
                message.contains("eo_frames"),
                "the error must name the pool: {message}"
            );
            assert!(
                !message.contains("dora_pool"),
                "nothing may invent a segment name: {message}"
            );
        }
    }

    /// A non-string is a different failure from a missing key: it means the
    /// daemon (or another binding) wrote the parameter with the wrong variant,
    /// which no amount of retrying will fix.
    #[test]
    fn a_segment_name_of_the_wrong_variant_is_reported_as_such() {
        let metadata = metadata_with(Some(DoraParameter::Integer(7)));
        let message = format!(
            "{:#}",
            pool_segment_name(&metadata, "p").expect_err("a non-string name is unusable")
        );
        assert!(
            message.contains("not a string"),
            "unexpected error: {message}"
        );
    }

    // -----------------------------------------------------------------
    // Consumer side: views over real segments
    //
    // These map real files in `/dev/shm`, a process-global namespace, so they
    // are Linux-only for the same platform reasons as `PoolSegment`'s own
    // tests: macOS caps a POSIX shm name at 31 bytes and neither macOS nor
    // Windows has the `/dev/shm` path `unlink` removes.
    // -----------------------------------------------------------------

    #[cfg(target_os = "linux")]
    mod views {
        use super::*;
        use dora_memory_pool::doradma;
        use dora_memory_pool::segment::Transport;
        use shared_memory_extended::ShmemConf;

        /// Owns a segment name and unlinks it on drop, so a panicking test
        /// cannot make its own name permanently unusable — `create` is
        /// `O_EXCL`.
        struct SegmentGuard {
            name: String,
            pool_id: String,
        }

        impl SegmentGuard {
            fn new(tag: &str) -> Self {
                static COUNTER: AtomicU64 = AtomicU64::new(0);
                let pool_id = format!(
                    "{tag}-{}-{}",
                    std::process::id(),
                    COUNTER.fetch_add(1, AtomicOrdering::Relaxed)
                );
                let name = naming::segment_name("cxxtest", "nodetest", &pool_id)
                    .expect("a valid segment name");
                Self { name, pool_id }
            }
        }

        impl Drop for SegmentGuard {
            fn drop(&mut self) {
                let _ = std::fs::remove_file(format!("/dev/shm/{}", self.name));
            }
        }

        /// A writer plus a view of the same pool, as a producer node and a
        /// consumer node would have.
        fn writer_and_view(
            tag: &str,
            size: usize,
        ) -> (SegmentGuard, PoolSegment, DoraMemoryPoolView) {
            let guard = SegmentGuard::new(tag);
            let writer = PoolSegment::create(&guard.name, size, "uint8", &[size], Transport::Shmem)
                .expect("create the pool");
            let view = open_view(&guard);
            (guard, writer, view)
        }

        fn open_view(guard: &SegmentGuard) -> DoraMemoryPoolView {
            register_view(
                PoolSegment::open(&guard.name).expect("open the pool"),
                guard.pool_id.clone(),
            )
        }

        /// The segment a Python sender writes for a CUDA receiver on a discrete
        /// GPU: header-only, `ipc_flag = 1`, and a JSON `size` describing a
        /// tensor in the IPC-imported device buffer — megabytes larger than the
        /// mapping itself.
        fn create_ipc_segment(guard: &SegmentGuard, declared_size: usize) {
            let json = format!(
                r#"{{"size":{declared_size},"dtype":"uint8","shape":[{declared_size}],"pinned_type":"cuda"}}"#
            );
            let total = doradma::data_offset_for(json.len());
            let mut shmem = ShmemConf::new()
                .os_id(&guard.name)
                .size(total)
                .writable(true)
                .create()
                .expect("create the ipc segment");
            shmem.set_owner(false);
            let buf = unsafe { std::slice::from_raw_parts_mut(shmem.as_ptr(), total) };
            doradma::write_header(buf, &json).expect("write the header");
            buf[doradma::OFFSET_IPC_FLAG..doradma::OFFSET_IPC_FLAG + 8]
                .copy_from_slice(&1u64.to_le_bytes());
        }

        #[test]
        fn a_view_of_a_live_pool_hands_out_its_mapping_and_its_payload() {
            let (_guard, writer, view) = writer_and_view("live", 64);
            let view = Box::new(view);

            assert!(view_is_alive(&view));
            let (mut base, mut bytes) = (0u64, 0usize);
            assert!(view_mapping(&view, &mut base, &mut bytes));
            assert_eq!(base, view.segment.shm_base());
            assert_eq!(bytes, writer.segment_bytes());

            let (mut ptr, mut len) = (0u64, 0usize);
            assert!(view_payload(&view, &mut ptr, &mut len));
            assert_eq!(len, 64);
            assert_eq!(view_payload_len(&view), 64);
            let mut offset = 0usize;
            assert!(view_payload_offset(&view, &mut offset));
            assert_eq!(ptr, base + offset as u64);
            assert!(
                ptr + len as u64 <= base + bytes as u64,
                "the payload must lie inside the mapping it is bounded by"
            );
            assert_eq!(view_transport(&view), "shmem");
            assert_eq!(view_dtype(&view), "uint8");
            assert_eq!(view_shape(&view), vec![64]);
            assert!(!view_ipc_present(&view));
            assert!(view_ipc_handle(&view).is_empty());
        }

        /// The whole point of the liveness flag: once the daemon says the pool
        /// is gone, nothing on this surface may hand out an address into it.
        #[test]
        fn a_freed_pool_hands_out_no_address_and_no_bytes() {
            let (guard, _writer, view) = writer_and_view("freed", 64);
            let view = Box::new(view);
            assert_eq!(view_payload_len(&view), 64, "alive to begin with");

            mark_freed(vec![guard.pool_id.clone()]);

            assert!(!view_is_alive(&view));
            let (mut base, mut bytes) = (0xdead_beefu64, usize::MAX);
            assert!(!view_mapping(&view, &mut base, &mut bytes));
            assert_eq!(
                (base, bytes),
                (0xdead_beef, usize::MAX),
                "a refused mapping must leave both outputs untouched, not zero one of them"
            );

            let (mut ptr, mut len) = (0xdead_beefu64, usize::MAX);
            assert!(!view_payload(&view, &mut ptr, &mut len));
            assert_eq!(
                (ptr, len),
                (0xdead_beef, usize::MAX),
                "a refused payload must leave both outputs untouched"
            );

            let mut offset = usize::MAX;
            assert!(!view_payload_offset(&view, &mut offset));
            assert_eq!(offset, usize::MAX);
            assert_eq!(view_payload_len(&view), 0);
            // Nothing about the ipc handle is asserted here: this pool never
            // had one, so the assertion would hold whatever the accessor did.
            // The rule is checked on the pool that has one instead.

            // A zero-length destination matches a zero-length payload, so the
            // length rule alone would let this through.
            let mut nothing: [u8; 0] = [];
            assert!(
                !view_try_read(&view, &mut nothing),
                "a dead pool must be refused before the length is even considered"
            );
            let mut buffer = [0u8; 64];
            assert!(!view_try_read(&view, &mut buffer));
        }

        /// The registry is keyed by pool id, and a free notification names one
        /// pool. Killing every view would take out pools the daemon never
        /// mentioned.
        #[test]
        fn freeing_one_pool_leaves_the_other_pools_views_alive() {
            let (freed_guard, _w1, freed_view) = writer_and_view("selective-a", 32);
            let (_kept_guard, _w2, kept_view) = writer_and_view("selective-b", 32);
            let (freed_view, kept_view) = (Box::new(freed_view), Box::new(kept_view));

            mark_freed(vec![freed_guard.pool_id.clone()]);

            assert!(!view_is_alive(&freed_view));
            assert!(view_is_alive(&kept_view), "an unrelated pool must survive");
            assert_eq!(view_payload_len(&kept_view), 32);
        }

        /// Every view of a freed pool dies, not just the first one: a node may
        /// hold several, and one surviving view is one live pointer into a
        /// released pool.
        #[test]
        fn freeing_a_pool_kills_every_view_of_it() {
            let (guard, _writer, first) = writer_and_view("multi", 32);
            let second = open_view(&guard);
            let (first, second) = (Box::new(first), Box::new(second));

            // The ids come back out, which is the only way `take_freed_pools`
            // can hand them to a node that has to `cudaHostUnregister` the
            // mappings — and it can only get them by having killed the views.
            assert_eq!(
                mark_freed(vec![guard.pool_id.clone()]),
                vec![guard.pool_id.clone()]
            );

            assert!(!view_is_alive(&first));
            assert!(!view_is_alive(&second));
        }

        #[test]
        fn dropping_a_view_unregisters_only_itself() {
            let (guard, _writer, first) = writer_and_view("dropreg", 32);
            let second = Box::new(open_view(&guard));
            assert_eq!(
                view_registry().get(&guard.pool_id).map(Vec::len),
                Some(2),
                "both views must be registered"
            );

            drop(first);
            assert_eq!(
                view_registry().get(&guard.pool_id).map(Vec::len),
                Some(1),
                "dropping one view must not unregister the other"
            );
            assert!(
                view_is_alive(&second),
                "the surviving view must still be alive"
            );

            drop(second);
            assert!(
                !view_registry().contains_key(&guard.pool_id),
                "the last view out must remove the entry, not leave an empty one behind"
            );
        }

        #[test]
        fn view_try_read_copies_a_frame_the_writer_finished() {
            let (_guard, mut writer, view) = writer_and_view("read", 64);
            writer.write(&[0xAB; 64]).expect("write a frame");
            let view = Box::new(view);

            let mut buffer = [0u8; 64];
            assert!(view_try_read(&view, &mut buffer));
            assert!(buffer.iter().all(|&byte| byte == 0xAB), "{buffer:?}");
        }

        /// The frame is being overwritten, so whatever is in it now is not a
        /// frame. The copy must be reported as unusable — this is the case the
        /// seqlock exists for.
        #[test]
        fn view_try_read_refuses_a_frame_a_writer_is_in_the_middle_of() {
            let (_guard, mut writer, view) = writer_and_view("torn", 64);
            writer.write(&[0xAB; 64]).expect("a complete frame first");
            let view = Box::new(view);

            writer.begin_write().expect("open a write cycle");
            let mut buffer = [0u8; 64];
            assert!(
                !view_try_read(&view, &mut buffer),
                "a frame under construction must not be handed on"
            );

            writer.end_write(true);
            assert!(
                view_try_read(&view, &mut buffer),
                "the frame must be readable again once the writer closes the cycle"
            );
        }

        /// A pool frame is fixed-size and nothing on the wire carries a
        /// per-frame length, so a destination of any other size cannot be
        /// filled with a frame.
        #[test]
        fn view_try_read_refuses_a_destination_that_is_not_the_frame_size() {
            let (_guard, mut writer, view) = writer_and_view("size", 64);
            writer.write(&[0xAB; 64]).expect("write a frame");
            let view = Box::new(view);

            let mut short = [0u8; 32];
            assert!(!view_try_read(&view, &mut short));
            assert!(
                short.iter().all(|&byte| byte == 0),
                "a refused read must not have copied a prefix: {short:?}"
            );

            let mut long = [0u8; 65];
            assert!(!view_try_read(&view, &mut long));
            assert!(long.iter().all(|&byte| byte == 0), "{long:?}");

            let mut exact = [0u8; 64];
            assert!(view_try_read(&view, &mut exact), "the frame size is 64");
        }

        #[test]
        fn a_zero_copy_read_is_valid_when_no_write_intervened() {
            let (_guard, mut writer, view) = writer_and_view("zerocopy", 32);
            writer.write(&[1; 32]).expect("write a frame");
            let view = Box::new(view);

            let read = view_begin_read(&view);
            assert!(view_read_valid(&view, &read));

            writer.write(&[2; 32]).expect("write another frame");
            assert!(
                !view_read_valid(&view, &read),
                "a frame written during the read must invalidate it"
            );
        }

        /// Two views of one segment share a generation word, so a token from
        /// the wrong view would compare equal and validate a bracket that
        /// never enclosed the read. Identity, not equality, is what rejects it.
        #[test]
        fn a_read_token_is_refused_by_a_view_that_did_not_issue_it() {
            let (guard, mut writer, first) = writer_and_view("token", 32);
            writer.write(&[3; 32]).expect("write a frame");
            let second = Box::new(open_view(&guard));
            let first = Box::new(first);

            let read = view_begin_read(&first);
            assert!(view_read_valid(&first, &read), "valid on its own view");
            assert!(
                !view_read_valid(&second, &read),
                "a token from another view must be refused, not compared"
            );
        }

        #[test]
        fn a_freed_pool_invalidates_a_read_that_was_already_open() {
            let (guard, mut writer, view) = writer_and_view("freedread", 32);
            writer.write(&[4; 32]).expect("write a frame");
            let view = Box::new(view);

            let read = view_begin_read(&view);
            assert!(view_read_valid(&view, &read));
            mark_freed(vec![guard.pool_id.clone()]);
            assert!(
                !view_read_valid(&view, &read),
                "a read of a released pool must not be reported as intact"
            );
        }

        /// The hazard the whole consumer surface is shaped around: an `ipc`
        /// pool's payload is in device memory, its segment holds only a header,
        /// and its declared size exceeds the entire mapping by megabytes. Every
        /// byte-reaching accessor must refuse it rather than hand out an
        /// address one past the end with a multi-megabyte length beside it.
        #[test]
        fn an_ipc_pool_view_hands_out_no_bytes_only_its_handle() {
            let guard = SegmentGuard::new("ipc");
            create_ipc_segment(&guard, 8 * 1024 * 1024);
            let view = Box::new(open_view(&guard));

            assert_eq!(view_transport(&view), "ipc");
            assert!(view_is_alive(&view), "an ipc pool is a live pool");

            let (mut ptr, mut len) = (0xdead_beefu64, usize::MAX);
            assert!(!view_payload(&view, &mut ptr, &mut len));
            assert_eq!((ptr, len), (0xdead_beef, usize::MAX));
            let mut offset = usize::MAX;
            assert!(!view_payload_offset(&view, &mut offset));
            assert_eq!(offset, usize::MAX);
            assert_eq!(view_payload_len(&view), 0);

            let mut buffer = [0u8; 4096];
            assert!(!view_try_read(&view, &mut buffer));
            let mut nothing: [u8; 0] = [];
            assert!(
                !view_try_read(&view, &mut nothing),
                "an ipc pool has no frame, so no destination size can match one"
            );

            // The mapping itself is real and registerable; only the payload is
            // elsewhere. And the handle is how a discrete-GPU consumer reaches
            // the payload that is not here.
            let (mut base, mut bytes) = (0u64, 0usize);
            assert!(view_mapping(&view, &mut base, &mut bytes));
            assert!(
                bytes < 8 * 1024 * 1024,
                "the point of this segment is that its declared size exceeds it: {bytes}"
            );
            assert!(view_ipc_present(&view));
            assert_eq!(view_ipc_handle(&view).len(), doradma::IPC_HANDLE_LEN);

            // The handle is not a pointer into this mapping, but it names a
            // device buffer that the free released, so it must stop being
            // handed out with everything else. This is the only pool in these
            // tests that has a handle at all, so it is the only place the rule
            // can be checked.
            mark_freed(vec![guard.pool_id.clone()]);
            assert!(
                view_ipc_handle(&view).is_empty(),
                "a released pool must not keep handing out a handle to its device buffer"
            );
        }

        /// The daemon's free notifications reach this binding through
        /// `dora_node_api::event_stream::memory_pool`. Nothing pushes into that
        /// set here — only the event thread does — so this pins the path and
        /// the rule that an empty drain kills nothing.
        #[test]
        fn take_freed_pools_is_empty_and_harmless_without_a_notification() {
            let (_guard, _writer, view) = writer_and_view("nofree", 32);
            let view = Box::new(view);

            let freed = take_freed_pools();
            assert!(
                !freed.iter().any(|id| id == &view.id),
                "nothing freed this pool: {freed:?}"
            );
            assert!(view_is_alive(&view));
            assert_eq!(view_payload_len(&view), 32);
        }
    }

    // -----------------------------------------------------------------
    // C++ header
    // -----------------------------------------------------------------

    /// `build.rs` compiles consumers with `-std=c++20`; GCC before 10 spells
    /// the same standard `-std=c++2a`. Probed on an empty translation unit so
    /// that a rejected flag is never confused with a broken header.
    #[cfg(unix)]
    fn cxx20_flag(compiler: &str) -> &'static str {
        for flag in ["-std=c++20", "-std=c++2a"] {
            let probe = std::process::Command::new(compiler)
                .args([flag, "-fsyntax-only", "-x", "c++", "/dev/null"])
                .output();
            if matches!(&probe, Ok(output) if output.status.success()) {
                return flag;
            }
        }
        panic!("`{compiler}` accepts neither `-std=c++20` nor `-std=c++2a`");
    }

    /// `dora/memory_pool.hpp` is shipped for a consumer to compile: `build.rs`
    /// runs `cxx_build::bridges` for its codegen only and never invokes a
    /// compiler, so nothing else in this workspace would notice the header
    /// breaking. Compiling one translation unit against the headers `build.rs`
    /// just installed is the cheapest thing that does.
    ///
    /// Unix only — it drives a `c++`-style command line, and the memory-pool
    /// transport is POSIX shared memory to begin with. The header is not
    /// covered on Windows.
    #[cfg(unix)]
    #[test]
    fn memory_pool_header_compiles() {
        let source = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests")
            .join("memory_pool_compile.cpp");
        let include_dir = env!("DORA_NODE_API_CXX_INCLUDE_DIR");
        let compiler = std::env::var("CXX").unwrap_or_else(|_| "c++".to_string());
        let std_flag = cxx20_flag(&compiler);

        let output = std::process::Command::new(&compiler)
            .arg(std_flag)
            // Full semantic analysis, so every `static_assert` and every body
            // in the translation unit is checked, without producing an object
            // nobody links.
            .arg("-fsyntax-only")
            .arg("-I")
            .arg(include_dir)
            .arg(&source)
            .output()
            .unwrap_or_else(|err| {
                panic!(
                    "failed to run the C++ compiler `{compiler}` on {}: {err}",
                    source.display()
                )
            });

        assert!(
            output.status.success(),
            "`{compiler}` rejected {} (headers from {include_dir}):\n{}",
            source.display(),
            String::from_utf8_lossy(&output.stderr)
        );
    }
}
