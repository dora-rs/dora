use crate::{
    DaemonCommunicationWrapper, EventStream, NodeError, NodeResult,
    daemon_connection::{DaemonChannel, IntegrationTestingEvents},
    integration_testing::{
        TestingCommunication, TestingInput, TestingOptions, TestingOutput,
        take_testing_communication,
    },
};

use self::{arrow_utils::ipc_encode, control_channel::ControlChannel};
use aligned_vec::{AVec, ConstAlign};
use arrow::array::{Array, ArrayData};
use colored::Colorize;
use dora_core::{
    config::{DataId, NodeId, NodeRunConfig},
    descriptor::{Descriptor, DescriptorExt},
    topics::{DORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT, DORA_DAEMON_LOCAL_LISTEN_PORT_ENV, LOCALHOST},
    types::TypeRegistry,
    uhlc,
};
use dora_message::{
    DataflowId,
    daemon_to_node::{DaemonCommunication, DaemonReply, NodeConfig},
    metadata::{
        FIN, FLUSH, FRAMING, FRAMING_ARROW_IPC, Metadata, MetadataParameters, Parameter,
        SCHEMA_HASH, SEGMENT_ID, SEQ, SESSION_ID,
    },
    node_to_daemon::{DaemonRequest, DataMessage, Timestamped},
};
use eyre::WrapErr;
use is_terminal::IsTerminal;

#[cfg(feature = "tracing")]
use std::sync::Mutex;
use std::{
    collections::{BTreeSet, HashMap},
    path::PathBuf,
    sync::Arc,
    time::{Duration, Instant},
};
#[cfg(feature = "tracing")]
use tokio::runtime::Handle;

#[cfg(feature = "tracing")]
use dora_tracing::{OtelGuard, TracingBuilder};
use tracing::{info, warn};

pub mod arrow_utils;
mod control_channel;

/// Runtime type checking mode, controlled by `DORA_RUNTIME_TYPE_CHECK` env var.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RuntimeTypeCheck {
    /// No runtime type checking (default).
    Off,
    /// Log warnings on type mismatches.
    Warn,
    /// Return errors on type mismatches.
    Error,
}

impl RuntimeTypeCheck {
    fn from_env() -> Self {
        match std::env::var("DORA_RUNTIME_TYPE_CHECK").as_deref() {
            Ok("error") => Self::Error,
            Ok("1" | "warn" | "true") => Self::Warn,
            Ok("") | Err(_) => Self::Off,
            Ok(other) => {
                tracing::warn!(
                    "unknown DORA_RUNTIME_TYPE_CHECK value \"{other}\", \
                     expected \"warn\" or \"error\"; disabling runtime type check"
                );
                Self::Off
            }
        }
    }
}

/// The data size threshold at which we start using shared memory.
///
/// Shared memory works by sharing memory pages. This means that the smallest
/// memory region that can be shared is one memory page, which is typically
/// 4KiB.
///
/// Using shared memory for messages smaller than the page size still requires
/// sharing a full page, so we have some memory overhead. We also have some
/// performance overhead because we need to issue multiple syscalls. For small
/// messages it is faster to send them over a traditional TCP stream (or similar).
///
/// This hardcoded threshold value specifies which messages are sent through
/// shared memory. Messages that are smaller than this threshold are sent through
/// TCP.
pub const ZERO_COPY_THRESHOLD: usize = 4096;

/// Per-output data-plane readiness for the startup no-loss barrier.
///
/// The zenoh data plane is direct node-to-node pub/sub, so a producer that
/// starts publishing before a consumer's subscription has propagated drops those
/// early samples. Until every expected subscriber has announced its subscription
/// (via a liveliness readiness token — see
/// [`dora_core::topics::zenoh_input_ready_liveliness_topic`]), the producer keeps
/// delivering over the reliable daemon path and only switches this output to the
/// fast zenoh path once all subscribers are confirmed wired. Over-counting keeps
/// an output on the (lossless) daemon path; it never drops messages.
struct ZenohOutputReadiness {
    /// Number of subscriber input-links this output has in the dataflow.
    expected: usize,
    /// Latches `true` once all expected subscribers are confirmed wired. Never
    /// reverts — once switched to the fast path we stay on it.
    ready: bool,
    /// Distinct readiness-token key expressions currently seen for this output
    /// (deduped). Maintained by the liveliness subscriber callback.
    alive: Arc<std::sync::Mutex<std::collections::HashSet<String>>>,
    /// Liveliness subscriber counting readiness tokens for this output. Kept
    /// alive so its callback keeps `alive` current; dropped with the node.
    _liveliness_sub: zenoh::pubsub::Subscriber<()>,
}

/// How long [`DoraNode::init`] waits for the direct-zenoh routes of this node's
/// outputs to connect before handing control to user code.
///
/// The daemon "all nodes ready" barrier already guarantees every subscriber is
/// *declared* before any node runs; this only waits out the residual zenoh
/// propagation between two already-declared endpoints, so it is normally
/// sub-second. On timeout (e.g. a zenoh data plane that can't connect while the
/// control plane can) the un-connected outputs simply start on the reliable
/// daemon path and upgrade to direct zenoh once their subscribers appear — the
/// dataflow is never blocked, and nothing is dropped.
const ZENOH_OUTPUT_CONNECT_TIMEOUT: Duration = Duration::from_secs(5);
/// Poll interval while waiting for output routes to connect at startup.
const ZENOH_OUTPUT_CONNECT_POLL_INTERVAL: Duration = Duration::from_millis(5);

/// Must exceed zenoh's internal 10s session close timeout, which is not
/// enforceable when zenoh's net runtime is wedged.
pub(crate) const ZENOH_TEARDOWN_TIMEOUT: Duration = Duration::from_secs(15);

/// Allows sending outputs and retrieving node information.
///
/// The main purpose of this struct is to send outputs via Dora. There are also functions available
/// for retrieving the node configuration.
pub struct DoraNode {
    id: NodeId,
    dataflow_id: DataflowId,
    node_config: NodeRunConfig,
    control_channel: ControlChannel,
    clock: Arc<uhlc::HLC>,

    /// Zenoh session for direct node-to-node pub/sub (data plane).
    /// `None` in interactive/testing mode.
    zenoh_session: Option<zenoh::Session>,
    /// Zenoh shared memory provider for zero-copy publishing.
    zenoh_shm_provider: Option<zenoh::shm::ShmProvider<zenoh::shm::PosixShmProviderBackend>>,
    /// Per-output zenoh publishers (lazily created on first send).
    /// `'static` is sound because zenoh `Publisher` internally holds `Arc<Session>`,
    /// so it doesn't borrow from the session field on this struct.
    /// Publishers must be dropped BEFORE the session (enforced in Drop impl).
    zenoh_publishers: HashMap<DataId, zenoh::pubsub::Publisher<'static>>,
    /// Per-output schema publishers on the `@schema` subtopic (lazily created on
    /// the first small message). Their cache retains the last schema so a
    /// late-joining subscriber fetches it via a history query, letting the data
    /// topic carry only schema-less batches. Dropped before the session, like
    /// [`zenoh_publishers`](Self::zenoh_publishers).
    zenoh_schema_publishers: HashMap<DataId, zenoh_ext::AdvancedPublisher<'static>>,
    /// Per-output schema-once state: the confirmed-published schema hash (so
    /// the schema is only re-published when it changes or a publish failed) and
    /// the time of the last full-stream send (for the periodic in-band refresh).
    zenoh_schema_state: HashMap<DataId, SchemaOnceState>,
    /// Threshold for using zenoh SHM vs inline bytes (default 4096).
    zenoh_zero_copy_threshold: usize,
    /// Per-output startup readiness for the direct zenoh data plane (lazily
    /// created on first send). See [`ZenohOutputReadiness`].
    zenoh_output_readiness: HashMap<DataId, ZenohOutputReadiness>,

    dataflow_descriptor: serde_yaml::Result<Descriptor>,
    warned_unknown_output: BTreeSet<DataId>,
    interactive: bool,
    restart_count: u32,

    /// Runtime type checking state. `None` when off (zero overhead).
    /// When `Some`, holds the mode (Warn/Error) and a map of output DataId -> expected Arrow DataType.
    runtime_type_checks: Option<(RuntimeTypeCheck, HashMap<DataId, arrow_schema::DataType>)>,

    /// Tokio runtime owned by the node. Populated only when no ambient
    /// runtime was available at init. Must drop after the zenoh session
    /// (which is drained explicitly at the top of [`Drop`]) so that any
    /// async cleanup triggered by session shutdown can still run.
    _owned_runtime: Option<tokio::runtime::Runtime>,
}

impl DoraNode {
    /// Initiate a node from environment variables set by the Dora daemon or fall back to
    /// interactive mode.
    ///
    /// This is the recommended initialization function for Dora nodes, which are spawned by
    /// Dora daemon instances. The daemon will set a `DORA_NODE_CONFIG` environment variable to
    /// configure the node.
    ///
    /// When the node is started manually without the `DORA_NODE_CONFIG` environment variable set,
    /// the initialization will fall back to [`init_interactive`](Self::init_interactive) if `stdin`
    /// is a terminal (detected through
    /// [`isatty`](https://www.man7.org/linux/man-pages/man3/isatty.3.html)).
    ///
    /// If the `DORA_NODE_CONFIG` environment variable is not set and `DORA_TEST_WITH_INPUTS` is
    /// set, the node will be initialized in integration test mode. See the
    /// [integration testing](crate::integration_testing) module for details.
    ///
    /// This function will also initialize the node in integration test mode when the
    /// [`setup_integration_testing`](crate::integration_testing::setup_integration_testing)
    /// function was called before. This takes precedence over all environment variables.
    ///
    /// ```no_run
    /// use dora_node_api::DoraNode;
    ///
    /// let (mut node, mut events) = DoraNode::init_from_env().expect("Could not init node.");
    /// ```
    pub fn init_from_env() -> NodeResult<(Self, EventStream)> {
        Self::init_from_env_inner(true)
    }

    /// Initialize the node from environment variables set by the Dora daemon; error if not set.
    ///
    /// This function behaves the same as [`init_from_env`](Self::init_from_env), but it does _not_
    /// fall back to [`init_interactive`](Self::init_interactive). Instead, an error is returned
    /// when the `DORA_NODE_CONFIG` environment variable is missing.
    pub fn init_from_env_force() -> NodeResult<(Self, EventStream)> {
        Self::init_from_env_inner(false)
    }

    fn init_from_env_inner(fallback_to_interactive: bool) -> NodeResult<(Self, EventStream)> {
        if let Some(testing_comm) = take_testing_communication() {
            let TestingCommunication {
                input,
                output,
                options,
            } = *testing_comm;
            return Self::init_testing(input, output, options);
        }

        // normal execution (started by dora daemon)
        match std::env::var("DORA_NODE_CONFIG") {
            Ok(raw) => {
                let node_config: NodeConfig =
                    serde_yaml::from_str(&raw).context("failed to deserialize node config")?;
                return Self::init(node_config);
            }
            Err(std::env::VarError::NotUnicode(_)) => {
                return Err(NodeError::Init(
                    "DORA_NODE_CONFIG env variable is not valid unicode".into(),
                ));
            }
            Err(std::env::VarError::NotPresent) => {} // continue trying other init methods
        };

        // node integration test mode
        match std::env::var("DORA_TEST_WITH_INPUTS") {
            Ok(raw) => {
                let input_file = PathBuf::from(raw);
                let output_file = match std::env::var("DORA_TEST_WRITE_OUTPUTS_TO") {
                    Ok(raw) => PathBuf::from(raw),
                    Err(std::env::VarError::NotUnicode(_)) => {
                        return Err(NodeError::Init(
                            "DORA_TEST_WRITE_OUTPUTS_TO env variable is not valid unicode".into(),
                        ));
                    }
                    Err(std::env::VarError::NotPresent) => {
                        input_file.with_file_name("outputs.jsonl")
                    }
                };
                let skip_output_time_offsets =
                    std::env::var_os("DORA_TEST_NO_OUTPUT_TIME_OFFSET").is_some();

                let input = TestingInput::FromJsonFile(input_file);
                let output = TestingOutput::ToFile(output_file);
                let options = TestingOptions {
                    skip_output_time_offsets,
                };

                return Self::init_testing(input, output, options);
            }
            Err(std::env::VarError::NotUnicode(_)) => {
                return Err(NodeError::Init(
                    "DORA_TEST_WITH_INPUTS env variable is not valid unicode".into(),
                ));
            }
            Err(std::env::VarError::NotPresent) => {} // continue trying other init methods
        }

        // interactive mode
        if fallback_to_interactive && std::io::stdin().is_terminal() {
            println!(
                "{}",
                "Starting node in interactive mode as DORA_NODE_CONFIG env variable is not set"
                    .green()
            );
            return Self::init_interactive();
        }

        // no run mode applicable
        Err(NodeError::Init(
            "DORA_NODE_CONFIG env variable is not set".into(),
        ))
    }

    /// Create a builder for configuring a node connection.
    ///
    /// Setting a `node_id` selects the dynamic-node path; without one, `build()`
    /// falls back to [`init_from_env`](Self::init_from_env). Use this builder
    /// when you need a custom daemon port — the other init functions cover the
    /// common cases. Source-compatible with upstream dora 0.5.x: `.dynamic()`
    /// is accepted (no-op) so code written against upstream still compiles.
    ///
    /// ```no_run
    /// use dora_node_api::DoraNode;
    /// use dora_node_api::dora_core::config::NodeId;
    ///
    /// let (mut node, mut events) = DoraNode::builder()
    ///     .node_id(NodeId::from("plot".to_string()))
    ///     .daemon_port(6789)
    ///     .build()
    ///     .expect("Could not init node");
    /// ```
    pub fn builder() -> DoraNodeBuilder {
        DoraNodeBuilder::default()
    }

    /// Initiate a node from a dataflow id and a node id.
    ///
    /// This initialization function should be used for [_dynamic nodes_](index.html#dynamic-nodes).
    ///
    /// ```no_run
    /// use dora_node_api::DoraNode;
    /// use dora_node_api::dora_core::config::NodeId;
    ///
    /// let (mut node, mut events) = DoraNode::init_from_node_id(NodeId::from("plot".to_string())).expect("Could not init node plot");
    /// ```
    ///
    pub fn init_from_node_id(node_id: NodeId) -> NodeResult<(Self, EventStream)> {
        Self::builder().node_id(node_id).build()
    }

    /// Dynamic initialization function for nodes that are sometimes used as dynamic nodes.
    ///
    /// This function first tries initializing the traditional way through
    /// [`init_from_env`][Self::init_from_env]. If this fails, it falls back to
    /// [`init_from_node_id`][Self::init_from_node_id].
    pub fn init_flexible(node_id: NodeId) -> NodeResult<(Self, EventStream)> {
        if std::env::var("DORA_NODE_CONFIG").is_ok() {
            info!(
                "Skipping {node_id} specified within the node initialization in favor of `DORA_NODE_CONFIG` specified by `dora start`"
            );
            Self::init_from_env()
        } else {
            Self::init_from_node_id(node_id)
        }
    }

    /// Initialize the node in a standalone mode that prompts for inputs on the terminal.
    ///
    /// Instead of connecting to a `dora daemon`, this interactive mode prompts for node inputs
    /// on the terminal. In this mode, the node is completely isolated from the dora daemon and
    /// other nodes, so it cannot be part of a dataflow.
    ///
    /// Note that this function will hang indefinitely if no input is supplied to the interactive
    /// prompt. So it should be only used through a terminal.
    ///
    /// Because of the above limitations, it is not recommended to use this function directly.
    /// Use [**`init_from_env`**](Self::init_from_env) instead, which supports both normal daemon
    /// connections and manual interactive runs.
    ///
    /// ## Example
    ///
    /// Run any node that uses `init_interactive` or [`init_from_env`](Self::init_from_env) directly
    /// from a terminal. The node will then start in "interactive mode" and prompt you for the next
    /// input:
    ///
    /// ```bash
    /// > cargo build -p rust-dataflow-example-node
    /// > target/debug/rust-dataflow-example-node
    /// hello
    /// Starting node in interactive mode as DORA_NODE_CONFIG env variable is not set
    /// Node asks for next input
    /// ? Input ID
    /// [empty input ID to stop]
    /// ```
    ///
    /// The `rust-dataflow-example-node` expects a `tick` input, so let's set the input ID to
    /// `tick`. Tick messages don't have any data, so we leave the "Data" empty when prompted:
    ///
    /// ```bash
    /// Node asks for next input
    /// > Input ID tick
    /// > Data
    /// tick 0, sending 0x943ed1be20c711a4
    /// node sends output random with data: PrimitiveArray<UInt64>
    /// [
    ///   10682205980693303716,
    /// ]
    /// Node asks for next input
    /// ? Input ID
    /// [empty input ID to stop]
    /// ```
    ///
    /// We see that both the `stdout` output of the node and also the output messages that it sends
    /// are printed to the terminal. Then we get another prompt for the next input.
    ///
    /// If you want to send an input with data, you can either send it as text (for string data)
    /// or as a JSON object (for struct, string, or array data). Other data types are not supported
    /// currently.
    ///
    /// Empty input IDs are interpreted as stop instructions:
    ///
    /// ```bash
    /// > Input ID
    /// given input ID is empty -> stopping
    /// Received stop
    /// Node asks for next input
    /// event channel was stopped -> returning empty event list
    /// node reports EventStreamDropped
    /// node reports closed outputs []
    /// node reports OutputsDone
    /// ```
    ///
    /// In addition to the node output, we see log messages for the different events that the node
    /// reports. After `OutputsDone`, the node should exit.
    ///
    /// ### JSON data
    ///
    /// In addition to text input, the `Data` prompt also supports JSON objects, which will be
    /// converted to Apache Arrow struct arrays:
    ///
    /// ```bash
    /// Node asks for next input
    /// > Input ID some_input
    /// > Data { "field_1": 42, "field_2": { "inner": "foo" } }
    /// ```
    ///
    /// This JSON data is converted to the following Arrow array:
    ///
    /// ```text
    /// StructArray
    /// -- validity: [valid, ]
    /// [
    ///   -- child 0: "field_1" (Int64)
    ///      PrimitiveArray<Int64>
    ///      [42,]
    ///   -- child 1: "field_2" (Struct([Field { name: "inner", data_type: Utf8, nullable: true, dict_id: 0, dict_is_ordered: false, metadata: {} }]))
    ///      StructArray
    ///      -- validity: [valid,]
    ///      [
    ///        -- child 0: "inner" (Utf8)
    ///        StringArray
    ///        ["foo",]
    ///      ]
    /// ]
    /// ```
    pub fn init_interactive() -> NodeResult<(Self, EventStream)> {
        #[cfg(feature = "tracing")]
        {
            TracingBuilder::new("node")
                .with_stdout("debug", false)
                .build()
                .wrap_err("failed to set up tracing subscriber")?;
        }

        let node_config = NodeConfig {
            dataflow_id: DataflowId::new_v4(),
            node_id: "test-node"
                .parse()
                .map_err(|e| NodeError::Init(format!("{e}")))?,
            run_config: NodeRunConfig {
                inputs: Default::default(),
                outputs: Default::default(),
                output_types: Default::default(),
                output_framing: Default::default(),
                input_types: Default::default(),
                shared_memory_pool_size: None,
            },
            daemon_communication: Some(DaemonCommunication::Interactive),
            dataflow_descriptor: serde_yaml::Value::Null,
            dynamic: false,
            write_events_to: None,
            restart_count: 0,
        };
        let (mut node, events) = Self::init(node_config)?;
        node.interactive = true;
        Ok((node, events))
    }

    /// Initializes a node in integration test mode.
    ///
    /// No connection to a dora daemon is made in this mode. Instead, inputs are read from the
    /// specified `TestingInput`, and outputs are written to the specified `TestingOutput`.
    /// Additional options for the testing mode can be specified through `TestingOptions`.
    ///
    /// It is recommended to use this function only within test functions.
    pub fn init_testing(
        input: TestingInput,
        output: TestingOutput,
        options: TestingOptions,
    ) -> NodeResult<(Self, EventStream)> {
        let node_config = NodeConfig {
            dataflow_id: DataflowId::new_v4(),
            node_id: "test-node"
                .parse()
                .map_err(|e| NodeError::Init(format!("{e}")))?,
            run_config: NodeRunConfig {
                inputs: Default::default(),
                outputs: Default::default(),
                output_types: Default::default(),
                output_framing: Default::default(),
                input_types: Default::default(),
                shared_memory_pool_size: None,
            },
            daemon_communication: None,
            dataflow_descriptor: serde_yaml::Value::Null,
            dynamic: false,
            write_events_to: None,
            restart_count: 0,
        };
        let testing_comm = TestingCommunication {
            input,
            output,
            options,
        };
        let (mut node, events) = Self::init_with_options(node_config, Some(testing_comm))?;
        node.interactive = true;
        Ok((node, events))
    }

    /// Internal initialization routine that should not be used outside of Dora.
    #[doc(hidden)]
    #[tracing::instrument]
    pub fn init(node_config: NodeConfig) -> NodeResult<(Self, EventStream)> {
        Self::init_with_options(node_config, None)
    }

    #[tracing::instrument(skip(testing_communication))]
    fn init_with_options(
        node_config: NodeConfig,
        testing_communication: Option<TestingCommunication>,
    ) -> NodeResult<(Self, EventStream)> {
        let NodeConfig {
            dataflow_id,
            node_id,
            run_config,
            daemon_communication,
            dataflow_descriptor,
            dynamic,
            write_events_to,
            restart_count,
        } = node_config;
        let clock = Arc::new(uhlc::HLC::default());
        let input_config = run_config.inputs.clone();

        let daemon_communication = match daemon_communication {
            Some(comm) => comm.into(),
            None => match testing_communication {
                Some(comm) => {
                    let TestingCommunication {
                        input,
                        output,
                        options,
                    } = comm;
                    let (sender, mut receiver) = tokio::sync::mpsc::channel(5);
                    let new_communication = DaemonCommunicationWrapper::Testing { channel: sender };
                    let mut events = IntegrationTestingEvents::new(input, output, options)?;
                    std::thread::spawn(move || {
                        while let Some((request, reply_sender)) = receiver.blocking_recv() {
                            let reply = events.request(&request);
                            if reply_sender
                                .send(reply.unwrap_or_else(|err| {
                                    DaemonReply::Result(Err(format!("{err:?}")))
                                }))
                                .is_err()
                            {
                                eprintln!("failed to send reply");
                            }
                        }
                    });
                    new_communication
                }
                None => {
                    return Err(NodeError::Init(
                        "no daemon communication method specified".into(),
                    ));
                }
            },
        };

        // Initialize zenoh session for direct node-to-node data plane.
        // Skip in interactive/testing mode (no daemon, no dataflow topology).
        let is_standard_mode = matches!(
            daemon_communication,
            DaemonCommunicationWrapper::Standard(_)
        );
        // Pool size priority: per-node YAML config > env var > built-in default.
        let shm_pool_size = run_config
            .shared_memory_pool_size
            .map(|bs| bs.as_bytes())
            .or_else(|| {
                std::env::var("DORA_NODE_SHM_POOL_SIZE")
                    .ok()
                    .and_then(|s| s.parse::<usize>().ok())
            })
            // 8 MB default — kept deliberately small so the pool fits a
            // constrained `/dev/shm` (Docker/Kubernetes commonly cap it at
            // 64 MB). A pool that doesn't fit backing memory was observed to
            // break large-output delivery entirely (the segment can't be backed
            // as it fills), so bumping this default is NOT a safe way to widen
            // the large-message pipeline. Throughput under large-message bursts
            // is handled instead by the non-blocking `GarbageCollect` alloc
            // policy (see `allocate_data_sample` / `zenoh_publish`): the producer
            // never stalls on a momentarily-full pool, it just copies via the
            // heap path. Raise this only alongside a matching `/dev/shm` (via
            // `shared_memory_pool_size` / `DORA_NODE_SHM_POOL_SIZE`) to keep more
            // large outputs zero-copy.
            .unwrap_or(8 * 1024 * 1024);
        let zenoh_zero_copy_threshold = std::env::var("DORA_ZERO_COPY_THRESHOLD")
            .ok()
            .and_then(|s| s.parse::<usize>().ok())
            .unwrap_or(ZERO_COPY_THRESHOLD);
        let (zenoh_session, zenoh_shm_provider, owned_runtime) = if !is_standard_mode {
            (None, None, None)
        } else {
            let (handle, owned_runtime) = match tokio::runtime::Handle::try_current() {
                Ok(handle) => (handle, None),
                Err(_) => {
                    let rt = tokio::runtime::Builder::new_multi_thread()
                        .enable_all()
                        .thread_name("dora-node-runtime")
                        .build()
                        .map_err(|e| {
                            NodeError::Init(format!("failed to create owned tokio runtime: {e}"))
                        })?;
                    let handle = rt.handle().clone();
                    (handle, Some(rt))
                }
            };
            // Use scope + spawn to avoid panicking when called from a tokio
            // worker thread (block_on panics in that context on
            // current-thread runtimes).
            let session = std::thread::scope(|s| {
                match s
                    .spawn(|| handle.block_on(dora_core::topics::open_zenoh_session(None)))
                    .join()
                {
                    Ok(Ok(session)) => Ok(session),
                    Ok(Err(e)) => Err(NodeError::Init(format!(
                        "failed to open zenoh session: {e:?}"
                    ))),
                    Err(_panic) => Err(NodeError::Init("zenoh session init panicked".into())),
                }
            })?;
            // SHM provider is best-effort: if the OS rejects the segment
            // allocation (e.g. `/dev/shm` exhausted in CI), fall back to
            // `None`. `send_output_sample` already publishes via heap
            // buffers when the provider is missing.
            let provider = {
                use zenoh::Wait;
                use zenoh::shm::{AllocAlignment, MemoryLayout, ShmProviderBuilder};

                let alignment =
                    AllocAlignment::new(crate::arrow_utils::ARROW_BUFFER_ALIGNMENT_EXPONENT)
                        .expect("ARROW_BUFFER_ALIGNMENT is a valid power-of-two alignment");
                let layout = shm_pool_size
                    .checked_next_multiple_of(crate::arrow_utils::ARROW_BUFFER_ALIGNMENT)
                    .and_then(|aligned| MemoryLayout::new(aligned, alignment).ok());

                match layout {
                    Some(layout) => match ShmProviderBuilder::default_backend(layout).wait() {
                        Ok(provider) => Some(provider),
                        Err(e) => {
                            warn!(
                                "failed to create zenoh SHM provider ({e}); \
                                 falling back to heap-buffered publishes"
                            );
                            None
                        }
                    },
                    None => {
                        warn!(
                            "invalid zenoh SHM pool size ({shm_pool_size}); \
                             falling back to heap-buffered publishes"
                        );
                        None
                    }
                }
            };
            (Some(session), provider, owned_runtime)
        };

        let event_stream = EventStream::init(
            dataflow_id,
            &node_id,
            &daemon_communication,
            input_config,
            &run_config.input_types,
            clock.clone(),
            write_events_to,
            zenoh_session.as_ref(),
            dynamic,
        )
        .wrap_err("failed to init event stream")?;
        let control_channel =
            ControlChannel::init(dataflow_id, &node_id, &daemon_communication, clock.clone())
                .wrap_err("failed to init control channel")?;
        let runtime_type_checks = match RuntimeTypeCheck::from_env() {
            RuntimeTypeCheck::Off => None,
            mode => {
                let registry = TypeRegistry::new();
                let mut checks = HashMap::new();
                for (id, urn) in &run_config.output_types {
                    match registry.resolve_arrow_type(urn) {
                        Some(dt) => {
                            checks.insert(id.clone(), dt);
                        }
                        None => {
                            if registry.resolve(urn).is_some() {
                                info!(
                                    "runtime type check: skipping complex type \"{urn}\" on output \"{id}\""
                                );
                            } else {
                                warn!(
                                    "runtime type check: unknown type URN \"{urn}\" on output \"{id}\""
                                );
                            }
                        }
                    }
                }
                Some((mode, checks))
            }
        };

        let mut node = Self {
            id: node_id,
            dataflow_id,
            node_config: run_config.clone(),
            control_channel,
            clock,
            zenoh_session,
            zenoh_shm_provider,
            zenoh_publishers: HashMap::new(),
            zenoh_schema_publishers: HashMap::new(),
            zenoh_schema_state: HashMap::new(),
            zenoh_zero_copy_threshold,
            zenoh_output_readiness: HashMap::new(),
            dataflow_descriptor: serde_yaml::from_value(dataflow_descriptor),
            warned_unknown_output: BTreeSet::new(),
            interactive: false,
            restart_count,
            runtime_type_checks,
            _owned_runtime: owned_runtime,
        };

        if dynamic {
            // Env vars from the dataflow descriptor are already injected by the
            // daemon at spawn time via `Command::env()`.  Setting them here with
            // `std::env::set_var` would be undefined behavior because the tokio
            // multi-threaded runtime is already running and other threads may
            // call `std::env::var` concurrently.
            //
            // If the node was started outside the daemon (manual dynamic node),
            // the user must set the required env vars before launching the
            // process.
            if let Ok(descriptor) = &node.dataflow_descriptor
                && let Some(env_vars) = descriptor
                    .nodes
                    .iter()
                    .find(|n| n.id == node.id)
                    .and_then(|n| n.env.as_ref())
            {
                for key in env_vars.keys() {
                    if std::env::var(key).is_err() {
                        warn!(
                            "env var `{key}` declared in dataflow descriptor is not set; \
                                 it should have been injected by the daemon at spawn time"
                        );
                    }
                }
            }
        }

        // Warm up the direct-zenoh data plane before handing control to user
        // code: declare output publishers and wait (bounded) until their
        // subscribers are wired, so the first `send_output` on a connected output
        // hits an established route. Fail-safe — un-connected outputs start on the
        // daemon path and upgrade later (see `warm_up_direct_outputs`).
        node.warm_up_direct_outputs();

        Ok((node, event_stream))
    }

    /// Check whether `output_id` is declared as an output of this node.
    ///
    /// Returns `true` if the output is declared (or this node is `interactive`,
    /// which has no static output declaration); `false` and emits a one-time
    /// warning if the output is unknown. Public so callers building higher-level
    /// send helpers (e.g. the Python `send_output_raw` zero-copy path) can
    /// validate before allocating a buffer.
    pub fn validate_output(&mut self, output_id: &DataId) -> bool {
        if !self.node_config.outputs.contains(output_id) && !self.interactive {
            if !self.warned_unknown_output.contains(output_id) {
                warn!("Ignoring output `{output_id}` not in node's output list.");
                self.warned_unknown_output.insert(output_id.clone());
            }
            false
        } else {
            true
        }
    }

    /// Send raw data from the node to the other nodes.
    ///
    /// We take a closure as an input to enable zero copy on send.
    ///
    /// ```no_run
    /// use dora_node_api::{DoraNode, MetadataParameters};
    /// use dora_core::config::DataId;
    ///
    /// let (mut node, mut events) = DoraNode::init_from_env().expect("Could not init node.");
    ///
    /// let output = DataId::from("output_id".to_owned());
    ///
    /// let data: &[u8] = &[0, 1, 2, 3];
    /// let parameters = MetadataParameters::default();
    ///
    /// node.send_output_raw(
    ///    output,
    ///    parameters,
    ///    data.len(),
    ///    |out| {
    ///         out.copy_from_slice(data);
    ///     }).expect("Could not send output");
    /// ```
    ///
    /// Ignores the output if the given `output_id` is not specified as node output in the dataflow
    /// configuration file.
    pub fn send_output_raw<F>(
        &mut self,
        output_id: DataId,
        parameters: MetadataParameters,
        data_len: usize,
        data: F,
    ) -> NodeResult<()>
    where
        F: FnOnce(&mut [u8]),
    {
        if !self.validate_output(&output_id) {
            return Ok(());
        };
        // The receiver expects a self-describing Arrow IPC stream. Build it in
        // place: pre-write the UInt8 IPC header into the (shared-memory) sample,
        // then let the caller write their bytes straight into the data region —
        // zero payload copies (and the SHM sample is moved into zenoh's `put`).
        let total = ipc_encode::uint8_ipc_len(data_len)
            .map_err(|e| NodeError::Output(format!("Arrow IPC encode: {e}")))?;
        let mut sample = self.allocate_data_sample(total)?;
        let offset = ipc_encode::encode_uint8_ipc_header(&mut sample, data_len)
            .map_err(|e| NodeError::Output(format!("Arrow IPC encode: {e}")))?;
        data(&mut sample[offset..offset + data_len]);

        let mut parameters = parameters;
        parameters.insert(
            FRAMING.to_string(),
            Parameter::String(FRAMING_ARROW_IPC.to_string()),
        );
        self.send_output_sample(output_id, parameters, Some(sample))
    }

    /// Sends the give Arrow array as an output message.
    ///
    /// Uses shared memory for efficient data transfer if suitable.
    ///
    /// This method might copy the message once to move it to shared memory.
    ///
    /// Ignores the output if the given `output_id` is not specified as node output in the dataflow
    /// configuration file.
    pub fn send_output(
        &mut self,
        output_id: DataId,
        parameters: MetadataParameters,
        data: impl Array,
    ) -> NodeResult<()> {
        if !self.validate_output(&output_id) {
            return Ok(());
        };

        let arrow_array = data.to_data();

        // Runtime type check (only when DORA_RUNTIME_TYPE_CHECK is set).
        //
        // Skip the check when this message carries pattern metadata
        // (`request_id`, `goal_id`, or `goal_status`). Service, action,
        // and streaming patterns legitimately multiplex multiple Arrow
        // schemas through a single output — a service server may reply
        // with different response shapes for different request types —
        // so a single declared Arrow type cannot cover all variants.
        // Non-pattern messages still get full validation
        // (dora-rs/adora#150).
        if let Some((mode, checks)) = &self.runtime_type_checks
            && let Some(expected) = checks.get(&output_id)
            && !carries_pattern_correlation(&parameters)
        {
            let actual = arrow_array.data_type();
            if actual != expected {
                let msg = format!(
                    "output \"{output_id}\": expected Arrow type {expected:?}, got {actual:?}"
                );
                match mode {
                    RuntimeTypeCheck::Error => {
                        return Err(NodeError::Output(msg));
                    }
                    RuntimeTypeCheck::Warn => {
                        warn!("type mismatch: {msg}");
                    }
                    RuntimeTypeCheck::Off => unreachable!(),
                }
            }
        }

        self.send_output_array(output_id, parameters, arrow_array)
    }

    /// IPC-encode `arrow_array` into a (shared-memory) sample and send it.
    ///
    /// Every data-plane payload is a self-describing Arrow IPC stream. Uses the
    /// hand-rolled 1-copy fast path when the array type is eligible, falling
    /// back to the official writer (one extra copy) otherwise. The shared send
    /// path for [`send_output`](Self::send_output) and
    /// [`send_output_raw`](Self::send_output_raw).
    fn send_output_array(
        &mut self,
        output_id: DataId,
        mut parameters: MetadataParameters,
        arrow_array: ArrayData,
    ) -> NodeResult<()> {
        parameters.insert(
            FRAMING.to_string(),
            Parameter::String(FRAMING_ARROW_IPC.to_string()),
        );

        let sample = match ipc_encode::ipc_fast_path_len(&arrow_array) {
            Some(len) => {
                let mut s = self.allocate_data_sample(len)?;
                ipc_encode::encode_ipc_into(&arrow_array, &mut s)
                    .map_err(|e| NodeError::Output(format!("Arrow IPC encode: {e}")))?;
                s
            }
            None => {
                let bytes = ipc_encode::encode_ipc_to_vec(&arrow_array)
                    .map_err(|e| NodeError::Output(format!("Arrow IPC encode: {e}")))?;
                let mut s = self.allocate_data_sample(bytes.len())?;
                s.copy_from_slice(&bytes);
                s
            }
        };

        self.send_output_sample(output_id, parameters, Some(sample))
            .wrap_err("failed to send output")?;

        Ok(())
    }

    /// Send the given raw byte data as output.
    ///
    /// Might copy the data once to move it into shared memory.
    ///
    /// Ignores the output if the given `output_id` is not specified as node output in the dataflow
    /// configuration file.
    pub fn send_output_bytes(
        &mut self,
        output_id: DataId,
        parameters: MetadataParameters,
        data_len: usize,
        data: &[u8],
    ) -> NodeResult<()> {
        if !self.validate_output(&output_id) {
            return Ok(());
        };
        // `send_output_raw` allocates a `data_len`-byte sample and the closure
        // below copies `data` into it. A mismatch would otherwise panic deep
        // inside `copy_from_slice` ("source slice length .. does not match
        // destination slice length .."); return a clear error instead.
        if data.len() != data_len {
            return Err(NodeError::Output(format!(
                "send_output_bytes: data_len ({data_len}) does not match data.len() ({})",
                data.len()
            )));
        }
        self.send_output_raw(output_id, parameters, data_len, |sample| {
            sample.copy_from_slice(data)
        })
    }

    /// Sends the given [`DataSample`] as output.
    ///
    /// The sample must already be a self-describing Arrow IPC stream (the
    /// `FRAMING_ARROW_IPC` parameter should be set). It is recommended to use a
    /// function like [`send_output`][Self::send_output] instead, which handles
    /// the encoding.
    ///
    /// Ignores the output if the given `output_id` is not specified as node output in the dataflow
    /// configuration file.
    pub fn send_output_sample(
        &mut self,
        output_id: DataId,
        mut parameters: MetadataParameters,
        sample: Option<DataSample>,
    ) -> NodeResult<()> {
        // `SCHEMA_HASH` is an internal wire-protocol key that only
        // `publish_schema_once` may set, and only for the schema-less batch it
        // belongs to. A stale value forwarded from an input's metadata (the
        // receive path strips it, but a recorded/hand-built parameter map can
        // still carry one) would make receivers route this output's full
        // self-describing stream to the schema-once decoder, hash-mismatch, and
        // silently drop it (dora-rs/dora#2366 review).
        parameters.remove(SCHEMA_HASH);
        // Auto-inject OpenTelemetry trace context when telemetry is enabled.
        // Uses the ambient OTel context, which is populated when the tracing
        // subscriber has an OpenTelemetry layer (e.g., via with_otlp_tracing).
        // Only trace/span IDs are propagated (via W3C TraceContext propagator).
        // OTel Baggage is NOT propagated to avoid leaking sensitive data across
        // node boundaries. If a user explicitly provides this key, it wins.
        #[cfg(feature = "tracing")]
        if !parameters.contains_key("open_telemetry_context") {
            let cx = opentelemetry::Context::current();
            let serialized = dora_tracing::telemetry::serialize_context(&cx);
            if !serialized.is_empty() {
                parameters.insert(
                    "open_telemetry_context".to_string(),
                    crate::Parameter::String(serialized),
                );
            }
        }

        let metadata = Metadata::from_parameters(self.clock.new_timestamp(), parameters);

        let finalized = sample.map(|sample| sample.finalize());

        // How a data-plane message should be delivered.
        enum Delivery {
            /// zenoh delivered the payload (or it was consumed by a failed SHM
            /// put); only the daemon's control-plane state needs syncing.
            Zenoh,
            /// Deliver via the daemon control channel. `None` is a metadata-only
            /// message with no payload.
            Daemon(Option<DataMessage>),
        }

        // Always publish data-plane messages via zenoh when a session is
        // available. The daemon control channel is only used as a fallback when
        // the zenoh session could not be opened (e.g. interactive/testing mode)
        // or when no subscriber matched in time. An SHM-backed sample is moved
        // straight into zenoh's `put` (no extra copy); only the daemon fallback
        // copies it out into a `DataMessage::Vec`.
        let delivery = match finalized {
            Some(finalized) if self.zenoh_session.is_some() => {
                tracing::trace!(
                    output = %output_id,
                    size = finalized.byte_len(),
                    "publishing via zenoh"
                );
                match self.zenoh_publish(&output_id, &metadata, finalized) {
                    Ok(PublishOutcome::Published) => Delivery::Zenoh,
                    Ok(PublishOutcome::NotPublished(sample)) => {
                        Delivery::Daemon(Some(sample.into_data_message()))
                    }
                    Err(e) => {
                        tracing::warn!(
                            "zenoh publish failed ({e}); message dropped \
                             (SHM payload consumed, no daemon fallback)"
                        );
                        Delivery::Zenoh
                    }
                }
            }
            Some(finalized) => Delivery::Daemon(Some(finalized.into_data_message())),
            None => Delivery::Daemon(None),
        };

        match delivery {
            Delivery::Zenoh => {
                // Keep the daemon's control-plane state in sync (input
                // deadlines, circuit-breaker recovery) without duplicating the
                // data payload that zenoh already delivered.
                self.control_channel
                    .report_output_sent(output_id.clone(), metadata)
                    .wrap_err_with(|| format!("failed to report output {output_id}"))?;
            }
            Delivery::Daemon(data) => {
                // The daemon/TCP path serializes the whole message; an oversized
                // IPC payload would otherwise fail deep in the transport with a
                // generic error. Reject it here with a clear, output-specific
                // message. Large payloads are expected to reach a zenoh
                // subscriber instead, which has no such limit.
                if let Some(DataMessage::Vec(v)) = &data
                    && v.len() > dora_message::MAX_MESSAGE_BYTES
                {
                    return Err(NodeError::Output(format!(
                        "output \"{output_id}\": IPC-encoded message is {} bytes, exceeding \
                         the {}-byte daemon transport limit (no matching zenoh subscriber to \
                         take the large payload)",
                        v.len(),
                        dora_message::MAX_MESSAGE_BYTES,
                    )));
                }
                self.control_channel
                    .send_message(output_id.clone(), metadata, data)
                    .wrap_err_with(|| format!("failed to send output {output_id}"))?;
            }
        }

        Ok(())
    }

    /// Report the given outputs IDs as closed.
    ///
    /// The node is not allowed to send more outputs with the closed IDs.
    ///
    /// Closing outputs early can be helpful to receivers.
    pub fn close_outputs(&mut self, outputs_ids: Vec<DataId>) -> NodeResult<()> {
        // Validate the whole batch before mutating any local state. Removing
        // outputs eagerly would leave the node's local output set out of sync
        // with the daemon if a later id is unknown: the early ones would be
        // gone locally, yet `report_closed_outputs` is skipped on error so the
        // daemon never learns about them.
        for output_id in &outputs_ids {
            if !self.node_config.outputs.contains(output_id) {
                return Err(NodeError::Output(format!("unknown output {output_id}")));
            }
        }
        for output_id in &outputs_ids {
            self.node_config.outputs.remove(output_id);
        }

        self.control_channel
            .report_closed_outputs(outputs_ids)
            .wrap_err("failed to report closed outputs to daemon")?;

        Ok(())
    }

    /// Whether the direct zenoh data plane is safe to use for `output_id` — i.e.
    /// every subscriber this output has in the dataflow has announced its
    /// subscription. Returns `false` while any expected subscriber is still
    /// missing, so the caller keeps using the reliable daemon path (no message is
    /// dropped during the startup subscription-establishment window).
    ///
    /// Lazily declares the per-output liveliness counter on first use. Once all
    /// subscribers are confirmed the result latches `true` and later calls are a
    /// cheap map lookup. If readiness tracking cannot be set up (no zenoh session
    /// or an invalid key) this returns `true` so the data plane is not stalled.
    fn ensure_output_ready(&mut self, output_id: &DataId) -> bool {
        use zenoh::Wait;

        if self
            .zenoh_output_readiness
            .get(output_id)
            .is_some_and(|r| r.ready)
        {
            return true;
        }

        if !self.zenoh_output_readiness.contains_key(output_id) {
            match self.declare_output_readiness(output_id) {
                Some(readiness) => {
                    self.zenoh_output_readiness
                        .insert(output_id.clone(), readiness);
                }
                // Tracking unavailable — don't stall the data plane.
                None => return true,
            }
        }

        let (expected, count) = {
            let readiness = self
                .zenoh_output_readiness
                .get(output_id)
                .expect("readiness entry just ensured");
            let count = readiness.alive.lock().map(|alive| alive.len()).unwrap_or(0);
            (readiness.expected, count)
        };

        // No subscribers to wait for: nothing can be lost, use the fast path.
        if expected == 0 {
            self.mark_output_ready(output_id);
            return true;
        }

        if count < expected {
            return false;
        }

        // Every readiness token is visible (all subscribers declared their
        // subscriptions). Confirm zenoh has actually forwarded a matching
        // subscription to this publisher before switching, so the very first
        // zenoh sample isn't published into a not-yet-wired route.
        let matched = self
            .zenoh_publishers
            .get(output_id)
            .and_then(|publisher| publisher.matching_status().wait().ok())
            .is_some_and(|status| status.matching());
        if matched {
            self.mark_output_ready(output_id);
            return true;
        }

        false
    }

    /// Latch the output's data plane as ready (idempotent).
    fn mark_output_ready(&mut self, output_id: &DataId) {
        if let Some(readiness) = self.zenoh_output_readiness.get_mut(output_id) {
            if !readiness.ready {
                tracing::debug!(
                    output = %output_id,
                    subscribers = readiness.expected,
                    "all subscribers wired; switching output to direct zenoh data plane"
                );
            }
            readiness.ready = true;
        }
    }

    /// Build the per-output readiness counter: resolve how many subscribers the
    /// output has (from the dataflow descriptor) and declare a liveliness
    /// subscriber that counts their readiness tokens. Returns `None` if there is
    /// no zenoh session or the counting subscription could not be declared.
    fn declare_output_readiness(&self, output_id: &DataId) -> Option<ZenohOutputReadiness> {
        use zenoh::Wait;

        let session = self.zenoh_session.as_ref()?;

        // Expected subscriber count from the global topology. A descriptor that
        // failed to parse (never the case for a daemon-spawned node) yields 0,
        // which just means "no barrier" — the pre-existing behaviour.
        let expected = match &self.dataflow_descriptor {
            Ok(descriptor) => descriptor
                .output_subscriber_count(&self.id, output_id)
                .unwrap_or_else(|e| {
                    tracing::warn!(
                        output = %output_id,
                        "failed to count subscribers ({e}); not gating the zenoh data plane"
                    );
                    0
                }),
            Err(_) => 0,
        };

        let prefix = dora_core::topics::zenoh_output_ready_liveliness_prefix(
            self.dataflow_id,
            &self.id,
            output_id,
        );
        let key_expr = match zenoh::key_expr::KeyExpr::new(prefix) {
            Ok(key) => key.into_owned(),
            Err(e) => {
                tracing::warn!(output = %output_id, "invalid readiness key ({e}); not gating the zenoh data plane");
                return None;
            }
        };

        let alive = Arc::new(std::sync::Mutex::new(std::collections::HashSet::new()));
        let alive_cb = alive.clone();
        // `history(true)` replays tokens that were declared before this
        // subscriber, so subscribers that wired up early are still counted.
        let subscriber = session
            .liveliness()
            .declare_subscriber(key_expr)
            .history(true)
            .callback(move |sample| {
                let key = sample.key_expr().as_str().to_string();
                if let Ok(mut alive) = alive_cb.lock() {
                    match sample.kind() {
                        zenoh::sample::SampleKind::Put => {
                            alive.insert(key);
                        }
                        zenoh::sample::SampleKind::Delete => {
                            alive.remove(&key);
                        }
                    }
                }
            })
            .wait();

        match subscriber {
            Ok(subscriber) => Some(ZenohOutputReadiness {
                expected,
                ready: false,
                alive,
                _liveliness_sub: subscriber,
            }),
            Err(e) => {
                tracing::warn!(output = %output_id, "failed to declare readiness subscriber ({e}); not gating the zenoh data plane");
                None
            }
        }
    }

    /// Ensure a direct-zenoh data publisher is declared for `output_id`.
    ///
    /// Idempotent: returns `true` if a publisher exists after the call (already
    /// present or freshly declared), `false` if there is no zenoh session or the
    /// declaration failed (the caller then falls back to the daemon path).
    ///
    /// QoS is configured at declare time so it applies to every put:
    /// `express(true)` bypasses zenoh's adaptive batch timer (the single biggest
    /// small-message latency win — without it, per-put delivery on the bare local
    /// config collapses to a few msg/s), `Priority::RealTime` keeps data-plane
    /// messages off the bulk-data queues, and `CongestionControl::Drop` prevents a
    /// stalled subscriber from back-pressuring the publishing node.
    fn ensure_zenoh_publisher(&mut self, output_id: &DataId) -> bool {
        use zenoh::Wait;
        use zenoh::qos::{CongestionControl, Priority};

        if self.zenoh_publishers.contains_key(output_id) {
            return true;
        }
        let Some(session) = self.zenoh_session.as_ref() else {
            return false;
        };
        let topic =
            dora_core::topics::zenoh_output_publish_topic(self.dataflow_id, &self.id, output_id);
        let key_expr = match zenoh::key_expr::KeyExpr::new(topic) {
            Ok(key) => key.into_owned(),
            Err(e) => {
                tracing::warn!(output = %output_id, "invalid zenoh key ({e}); falling back to daemon path");
                return false;
            }
        };
        let publisher = match session
            .declare_publisher(key_expr)
            .congestion_control(CongestionControl::Drop)
            .express(true)
            .priority(Priority::RealTime)
            .wait()
        {
            Ok(publisher) => publisher,
            Err(e) => {
                tracing::warn!(output = %output_id, "failed to declare zenoh publisher ({e}); falling back to daemon path");
                return false;
            }
        };
        self.zenoh_publishers.insert(output_id.clone(), publisher);
        true
    }

    /// Eagerly declare this node's direct-zenoh output publishers and readiness
    /// counters, then wait (bounded) until every output's subscribers are wired
    /// up — so the user's first `send_output` on a connected output is guaranteed
    /// to hit an established route rather than dropping startup samples.
    ///
    /// Runs once, at the end of [`Self::init`], *after* the daemon "all nodes
    /// ready" barrier. By then every subscriber in the dataflow has already
    /// declared its data subscriber and readiness token (the barrier releases
    /// only once all nodes have subscribed), so this merely waits out zenoh route
    /// propagation between already-declared endpoints — normally milliseconds.
    ///
    /// Fail-safe: outputs that don't connect within
    /// [`ZENOH_OUTPUT_CONNECT_TIMEOUT`] (an unreachable remote subscriber, or a
    /// zenoh data plane that can't connect while the control plane can) are left
    /// on the reliable daemon path and upgrade to direct zenoh on a later send
    /// once their subscribers appear. The dataflow is never blocked from starting
    /// and nothing is dropped.
    fn warm_up_direct_outputs(&mut self) {
        if self.zenoh_session.is_none() {
            return;
        }
        let outputs: Vec<DataId> = self.node_config.outputs.iter().cloned().collect();
        if outputs.is_empty() {
            return;
        }

        // Declare publishers up front so zenoh starts wiring routes immediately
        // and `ensure_output_ready` (which reads `matching_status`) can observe
        // them. `ensure_output_ready` lazily declares the readiness counter.
        for output_id in &outputs {
            self.ensure_zenoh_publisher(output_id);
        }

        let deadline = Instant::now() + ZENOH_OUTPUT_CONNECT_TIMEOUT;
        loop {
            let all_ready = outputs
                .iter()
                .all(|output_id| self.ensure_output_ready(output_id));
            if all_ready {
                break;
            }
            if Instant::now() >= deadline {
                let pending: Vec<&str> = outputs
                    .iter()
                    .filter(|o| !self.ensure_output_ready(o))
                    .map(|o| o.as_str())
                    .collect();
                tracing::debug!(
                    outputs = ?pending,
                    "direct-zenoh routes not confirmed within {}s; these outputs start on the daemon path and upgrade when their subscribers connect",
                    ZENOH_OUTPUT_CONNECT_TIMEOUT.as_secs()
                );
                break;
            }
            std::thread::sleep(ZENOH_OUTPUT_CONNECT_POLL_INTERVAL);
        }
    }

    /// Publish data directly via zenoh (node-to-node, bypassing daemon for data).
    /// Uses SHM for zero-copy when possible, falls back to heap buffer.
    ///
    /// The publisher is declared (see [`Self::ensure_zenoh_publisher`]) with
    /// `express(true)` to bypass zenoh's adaptive batch timer and with
    /// `Priority::RealTime` so data-plane messages don't share queues with bulk
    /// traffic.
    fn zenoh_publish(
        &mut self,
        output_id: &DataId,
        metadata: &Metadata,
        finalized: FinalizedSample,
    ) -> eyre::Result<PublishOutcome> {
        use zenoh::Wait;

        // Every failure *before* the payload is moved into `put` returns the
        // sample as `NotPublished` so the caller can still deliver it via the
        // daemon. Only a failed `put` of an SHM buffer (which consumes it)
        // returns `Err` — that is the single non-recoverable case.
        //
        // Get or create the publisher for this output. Normally it was already
        // declared eagerly at startup (see [`Self::warm_up_direct_outputs`]);
        // this covers outputs sent that weren't declared then. A missing
        // session or a declaration failure falls back to the daemon path.
        if !self.ensure_zenoh_publisher(output_id) {
            return Ok(PublishOutcome::NotPublished(finalized));
        }

        // Startup no-loss barrier: until every subscriber for this output has
        // wired up its zenoh subscription, deliver over the reliable daemon path
        // rather than dropping early samples on the not-yet-established data
        // plane. Once all subscribers are confirmed this latches ready and every
        // subsequent send takes the fast zenoh path.
        if !self.ensure_output_ready(output_id) {
            return Ok(PublishOutcome::NotPublished(finalized));
        }

        let Some(publisher) = self.zenoh_publishers.get(output_id) else {
            tracing::warn!(output = %output_id, "zenoh publisher missing; falling back to daemon path");
            return Ok(PublishOutcome::NotPublished(finalized));
        };
        let session = self
            .zenoh_session
            .as_ref()
            .expect("zenoh session presence checked above");

        // Serialize metadata as zenoh attachment.
        let metadata_bytes = match bincode::serialize(metadata) {
            Ok(bytes) => bytes,
            Err(e) => {
                tracing::warn!(output = %output_id, "failed to serialize metadata ({e}); falling back to daemon path");
                return Ok(PublishOutcome::NotPublished(finalized));
            }
        };

        match finalized {
            // The producer already wrote into shared memory. Move the SHM
            // buffer straight into `put` — no realloc, no copy. This is the
            // path that eliminates the former heap-to-SHM second copy.
            //
            // On a put error the buffer has been consumed and cannot be
            // recovered for the daemon fallback, so this returns an error
            // (the caller logs and drops the message). This is a deliberate
            // trade-off for zero-copy on the common matched-subscriber path.
            // Producer-constructed SHM sample: `put` *moves* (consumes) the SHM
            // buffer, so — unlike the borrowed-heap `Vec` arm below — there is no
            // intact payload left to retry on error. A put failure is therefore
            // best-effort: the message is dropped (the caller logs it). This is
            // the deliberate, accepted trade-off for the zero-copy large-output
            // path, not an oversight.
            FinalizedSample::Shm(sbuf) => {
                publisher
                    .put(sbuf)
                    .attachment(&metadata_bytes[..])
                    .wait()
                    .map_err(|e| eyre::eyre!("zenoh SHM publish failed: {e}"))?;
                Ok(PublishOutcome::Published)
            }
            // Heap payload. At or above the threshold, copy once into a fresh
            // SHM buffer so local subscribers still get zero-copy delivery;
            // below it, a heap-buffered put is cheaper than a full SHM page.
            // The heap buffer is only borrowed, so any put error can fall back
            // to the daemon path with the payload intact.
            FinalizedSample::Vec(avec) => {
                if avec.len() >= self.zenoh_zero_copy_threshold
                    && let Some(provider) = &self.zenoh_shm_provider
                {
                    use zenoh::shm::GarbageCollect;
                    // Non-blocking: garbage-collect freed chunks and allocate, but
                    // do NOT block waiting for the pool to drain. Under a burst of
                    // large messages the zero-copy receiver pins each segment for
                    // the whole receive pipeline, so the pool can be momentarily
                    // exhausted; `BlockOn` would then sleep 1 ms per retry (zenoh
                    // 1.8 has no alloc signalling yet), throttling throughput to
                    // ~1k msg/s. Falling back to a heap-buffered put instead keeps
                    // the producer moving (PR #2366).
                    match provider
                        .alloc(avec.len())
                        .with_policy::<GarbageCollect>()
                        .wait()
                    {
                        Ok(mut sbuf) => {
                            sbuf.as_mut().copy_from_slice(&avec);
                            return match publisher.put(sbuf).attachment(&metadata_bytes[..]).wait()
                            {
                                Ok(()) => Ok(PublishOutcome::Published),
                                Err(e) => {
                                    tracing::warn!(
                                        "zenoh SHM publish failed ({e}); \
                                         falling back to daemon path"
                                    );
                                    Ok(PublishOutcome::NotPublished(FinalizedSample::Vec(avec)))
                                }
                            };
                        }
                        Err(e) => {
                            tracing::debug!("SHM alloc failed ({e}), using heap buffer");
                        }
                    }
                }

                // A large payload that did not make it into SHM (no provider, or
                // the pool was momentarily full) must NOT be published over the
                // zenoh data plane: a payload larger than the transport batch
                // size is fragmented, and the express/`Drop` data publisher
                // silently drops fragmented messages — `put` reports success but
                // the subscriber never receives them (PR #2366). Route it via the
                // reliable daemon path instead (TCP, up to `MAX_MESSAGE_BYTES`).
                // Only sub-threshold payloads, which fit a single batch and never
                // fragment, take the zenoh heap put below.
                if avec.len() >= self.zenoh_zero_copy_threshold {
                    return Ok(PublishOutcome::NotPublished(FinalizedSample::Vec(avec)));
                }

                // Only sub-threshold (single-batch, never-fragmented) payloads
                // reach this point — large payloads were routed to the daemon
                // path above. Apply the schema-once optimization to small
                // messages with a stable Arrow schema: publish the schema on the
                // `@schema` subtopic (only on change) and send just the
                // schema-less batch on the data topic, tagged with the schema
                // hash so the receiver matches it to the decoder primed from the
                // subtopic.
                //
                // The message that (re)publishes the schema — the output's first,
                // every schema change, any message after a failed `@schema` put,
                // and a periodic refresh — is itself sent as a full
                // self-describing stream (`publish_schema_once` returns `None`
                // for it). It decodes standalone and primes receivers in-band,
                // in data-plane order, so the express batch can never outrun its
                // own schema (the `@schema` plane's non-express `Block` publish
                // otherwise loses that race) and a one-shot output cannot lose
                // its only message.
                //
                // Service/action request-reply messages (carrying
                // `request_id`/`goal_id`/`goal_status`) are excluded: a server
                // legitimately multiplexes multiple response schemas through one
                // output, interleaved per request, and each per-message schema
                // change would force a full stream + `@schema` publish anyway.
                // Sending them as full self-describing streams (the pre-PR
                // behavior) makes each message decode standalone regardless of
                // schema order, at the cost of ~400 B of framing per message —
                // acceptable for these request/reply-rate patterns.
                //
                // Streaming (`session_id`/`segment_id`) is deliberately NOT
                // excluded: every chunk of a stream shares one schema, so
                // schema-once primes once and each chunk reuses it — streaming is
                // the high-rate small-message case schema-once exists for. A
                // schema change at a segment boundary is just the one-time
                // re-prime window any schema-once output has, not the per-message
                // alternation that makes service/action lossy.
                //
                // `schema_once` is bound here, not inside the match, so its
                // attachment bytes outlive the `put` below.
                let schema_once = if schema_once_eligible(
                    avec.len(),
                    self.zenoh_zero_copy_threshold,
                    &metadata.parameters,
                ) {
                    publish_schema_once(
                        &mut self.zenoh_schema_publishers,
                        &mut self.zenoh_schema_state,
                        session,
                        self.dataflow_id,
                        &self.id,
                        output_id,
                        &avec,
                        metadata,
                    )
                } else {
                    None
                };
                // Fall back to a full standalone stream if the batch slice can't
                // be taken (a real IPC stream always can — defensive).
                let (payload, attachment): (&[u8], &[u8]) = match schema_once.as_ref() {
                    Some(att) => match arrow_utils::ipc_encode::batch_slice(&avec) {
                        Some(slice) => (slice, att.as_slice()),
                        None => (&avec[..], &metadata_bytes[..]),
                    },
                    None => (&avec[..], &metadata_bytes[..]),
                };
                match publisher.put(payload).attachment(attachment).wait() {
                    Ok(()) => Ok(PublishOutcome::Published),
                    Err(e) => {
                        tracing::warn!("zenoh publish failed ({e}); falling back to daemon path");
                        // The zenoh data plane did not deliver this message. If
                        // it was the one meant to prime receivers in-band (the
                        // first message of a schema, or a periodic refresh),
                        // `publish_schema_once` already recorded its state and
                        // the following messages would go out schema-less with
                        // no delivered priming stream. Forget the output's
                        // schema-once state so the next message sends a full
                        // stream and re-publishes the schema. (A congestion
                        // drop reports `Ok` and stays undetectable — inherent
                        // to `CongestionControl::Drop`; the periodic refresh
                        // bounds that residual window.)
                        self.zenoh_schema_state.remove(output_id);
                        Ok(PublishOutcome::NotPublished(FinalizedSample::Vec(avec)))
                    }
                }
            }
        }
    }

    /// Returns the ID of the node as specified in the dataflow configuration file.
    pub fn id(&self) -> &NodeId {
        &self.id
    }

    /// Returns the unique identifier for the running dataflow instance.
    ///
    /// Dora assigns each dataflow instance a random identifier when started.
    pub fn dataflow_id(&self) -> &DataflowId {
        &self.dataflow_id
    }

    /// Returns the input and output configuration of this node.
    pub fn node_config(&self) -> &NodeRunConfig {
        &self.node_config
    }

    /// Returns the zero-copy SHM threshold in bytes.
    ///
    /// Outputs whose raw payload is at least this many bytes are published via
    /// zenoh shared memory (zero-copy for local subscribers); smaller outputs
    /// are published via zenoh with a heap-buffered put. Configured via the
    /// `DORA_ZERO_COPY_THRESHOLD` env var, defaulting to
    /// [`ZERO_COPY_THRESHOLD`].
    pub fn zero_copy_threshold(&self) -> usize {
        self.zenoh_zero_copy_threshold
    }

    /// Returns true if this node was restarted after a previous exit or failure.
    ///
    /// Nodes can use this to decide whether to restore saved state or start fresh.
    pub fn is_restart(&self) -> bool {
        self.restart_count > 0
    }

    /// Returns how many times this node has been restarted.
    ///
    /// Returns 0 on the first run, 1 after the first restart, etc.
    pub fn restart_count(&self) -> u32 {
        self.restart_count
    }

    /// Returns the current timestamp from the node's Hybrid Logical Clock.
    ///
    /// This generates a new HLC timestamp, which combines the physical
    /// wall-clock time with a logical counter to ensure uniqueness and
    /// monotonicity even across nodes. The HLC is the same clock dora
    /// stamps every outgoing message with, so this is the right value
    /// to subtract from an input event's `metadata.timestamp` when
    /// measuring per-event processing latency — using
    /// `std::time::SystemTime::now()` instead would mix two unrelated
    /// clocks and give meaningless results across daemons.
    pub fn timestamp(&self) -> uhlc::Timestamp {
        self.clock.new_timestamp()
    }

    /// Send a structured log message.
    ///
    /// Outputs a JSONL line to stdout that the daemon parses automatically.
    /// Works with `min_log_level` filtering and `send_logs_as` routing.
    ///
    /// `level` should be one of: `"error"`, `"warn"`, `"info"`, `"debug"`, `"trace"`.
    /// Unknown levels default to `"info"`.
    pub fn log(&self, level: &str, message: &str, target: Option<&str>) {
        self.log_with_fields(level, message, target, None);
    }

    /// Maximum total size of log fields before they are dropped (60 KB).
    /// Matches the downstream 64 KB parse limit with headroom for the message envelope.
    const MAX_LOG_FIELDS_BYTES: usize = 60 * 1024;

    /// Send a structured log message with optional key-value fields.
    ///
    /// Like [`log`](Self::log), but accepts additional structured fields that
    /// are included in the JSON payload and preserved through `send_logs_as`.
    pub fn log_with_fields(
        &self,
        level: &str,
        message: &str,
        target: Option<&str>,
        fields: Option<&std::collections::BTreeMap<String, String>>,
    ) {
        let level_str = match level.to_lowercase().as_str() {
            "error" => "error",
            "warn" | "warning" => "warn",
            "info" => "info",
            "debug" => "debug",
            "trace" => "trace",
            _ => "info",
        };
        let timestamp = chrono::Utc::now().to_rfc3339();
        let mut entry = serde_json::json!({
            "timestamp": timestamp,
            "level": level_str,
            "node_id": self.id.to_string(),
            "message": message,
        });
        if let Some(target) = target {
            entry["target"] = serde_json::Value::String(target.to_string());
        }
        if let Some(fields) = fields {
            let total: usize = fields.iter().map(|(k, v)| k.len() + v.len()).sum();
            if total <= Self::MAX_LOG_FIELDS_BYTES {
                entry["fields"] = serde_json::json!(fields);
            } else {
                eprintln!("dora log: fields too large ({total} bytes), dropping fields");
                entry["fields_dropped"] = serde_json::Value::Bool(true);
            }
        }
        match serde_json::to_string(&entry) {
            Ok(json) => println!("{json}"),
            Err(e) => eprintln!("dora log serialization error: {e}"),
        }
    }

    /// Log an error message.
    pub fn log_error(&self, message: &str) {
        self.log("error", message, None);
    }

    /// Log a warning message.
    pub fn log_warn(&self, message: &str) {
        self.log("warn", message, None);
    }

    /// Log an info message.
    pub fn log_info(&self, message: &str) {
        self.log("info", message, None);
    }

    /// Log a debug message.
    pub fn log_debug(&self, message: &str) {
        self.log("debug", message, None);
    }

    /// Log a trace message.
    pub fn log_trace(&self, message: &str) {
        self.log("trace", message, None);
    }

    // -----------------------------------------------------------------
    // Service / Action helpers
    // -----------------------------------------------------------------

    /// Generate a new unique request/goal ID (UUID v7, time-ordered).
    ///
    /// Uses a per-thread monotonic counter context to guarantee uniqueness
    /// even when multiple IDs are generated within the same clock tick.
    pub fn new_request_id() -> String {
        thread_local! {
            static CTX: uuid::ContextV7 = const { uuid::ContextV7::new() };
        }
        CTX.with(|ctx| uuid::Uuid::new_v7(uuid::Timestamp::now(ctx)).to_string())
    }

    /// Generate a new unique goal ID (UUID v7, time-ordered).
    ///
    /// This is an alias for [`new_request_id`](Self::new_request_id) that
    /// reads more naturally in action (goal/feedback/result) contexts.
    pub fn new_goal_id() -> String {
        Self::new_request_id()
    }

    /// Send a service request, automatically injecting a `request_id` into the
    /// metadata parameters. Returns the generated request ID.
    ///
    /// Any existing `request_id` key in `parameters` is replaced.
    pub fn send_service_request(
        &mut self,
        output_id: DataId,
        mut parameters: MetadataParameters,
        data: impl Array,
    ) -> NodeResult<String> {
        if parameters.contains_key(dora_message::metadata::REQUEST_ID) {
            tracing::warn!("send_service_request: caller-provided request_id will be overwritten");
        }
        let request_id = Self::new_request_id();
        parameters.insert(
            dora_message::metadata::REQUEST_ID.to_string(),
            dora_message::metadata::Parameter::String(request_id.clone()),
        );
        self.send_output(output_id, parameters, data)?;
        Ok(request_id)
    }

    /// Send a service response. This is a semantic alias for [`send_output`](Self::send_output).
    ///
    /// The caller is expected to pass through the `request_id` parameter from
    /// the incoming request's metadata.
    pub fn send_service_response(
        &mut self,
        output_id: DataId,
        parameters: MetadataParameters,
        data: impl Array,
    ) -> NodeResult<()> {
        self.send_output(output_id, parameters, data)
    }

    // -----------------------------------------------------------------
    // Streaming helpers
    // -----------------------------------------------------------------

    /// Send a streaming segment chunk. Convenience wrapper around
    /// [`send_output`](Self::send_output) that builds metadata from the
    /// [`StreamSegment`] builder.
    pub fn send_stream_chunk(
        &mut self,
        output_id: DataId,
        segment: &mut StreamSegment,
        fin: bool,
        data: impl Array,
    ) -> NodeResult<()> {
        self.send_output(output_id, segment.chunk(fin), data)
    }

    /// Allocates a [`DataSample`] of the specified size.
    ///
    /// For payloads at or above the zero-copy threshold the buffer is allocated
    /// directly from the zenoh SHM provider (when available), so the producer
    /// writes straight into shared memory and publishing moves the buffer into
    /// zenoh's `put` without a further copy. Smaller payloads — or the case
    /// where no SHM provider exists (interactive/testing mode) — use a
    /// heap-allocated, 128-byte-aligned buffer; the SHM provider is
    /// page-aligned, so dedicating a full page to a small message is pure waste.
    pub fn allocate_data_sample(&mut self, data_len: usize) -> NodeResult<DataSample> {
        if data_len >= self.zenoh_zero_copy_threshold
            && let Some(provider) = &self.zenoh_shm_provider
        {
            use zenoh::Wait;
            use zenoh::shm::GarbageCollect;
            // Non-blocking (see `zenoh_publish`): GC and allocate, but fall back
            // to a heap buffer rather than `BlockOn`-sleeping 1 ms when the pool
            // is momentarily full under a large-message burst. The heap buffer
            // costs one extra copy on publish but keeps the producer from
            // stalling, which is what regressed sustained throughput (PR #2366).
            match provider
                .alloc(data_len)
                .with_policy::<GarbageCollect>()
                .wait()
            {
                Ok(sbuf) => {
                    // Use the SHM buffer only when it is exactly the requested
                    // size — zenoh 1.8 guarantees this (the logical length
                    // matches the request even when the backing chunk is
                    // larger). If a future provider ever over-allocates, fall
                    // back to heap rather than expose or publish an oversized
                    // slice (`DataSample` has no length cap of its own).
                    if sbuf.as_ref().len() == data_len {
                        return Ok(DataSample {
                            storage: SampleStorage::Shm(sbuf),
                        });
                    }
                    tracing::debug!(
                        "zenoh SHM alloc returned {} bytes for a {data_len}-byte \
                         request; using heap",
                        sbuf.as_ref().len()
                    );
                }
                Err(e) => {
                    tracing::debug!("SHM alloc failed ({e}), using heap buffer");
                }
            }
        }

        let avec: AVec<u8, ConstAlign<128>> = AVec::__from_elem(128, 0, data_len);
        Ok(avec.into())
    }

    /// Returns the full dataflow descriptor that this node is part of.
    ///
    /// This method returns the parsed dataflow YAML file.
    pub fn dataflow_descriptor(&self) -> NodeResult<&Descriptor> {
        match &self.dataflow_descriptor {
            Ok(d) => Ok(d),
            Err(err) => Err(NodeError::Data(format!(
                "failed to parse dataflow descriptor: {err}\n\n\
                    This might be caused by mismatched version numbers of dora \
                    daemon and the dora node API"
            ))),
        }
    }

    /// Register a pinned memory pool with the daemon for lifecycle tracking.
    ///
    /// Send the memory pool metadata to the daemon so it can track the pool
    /// and provide it to other nodes for zero-copy access.
    pub fn register_pinned_memory(
        &mut self,
        shared_memory_id: String,
        metadata: Metadata,
    ) -> Result<(), eyre::Error> {
        self.control_channel
            .register_pinned_memory(shared_memory_id, metadata)
    }

    /// Read pinned memory metadata from the daemon.
    ///
    /// When `free` is true, the daemon also frees the pool after reading.
    pub fn read_pinned_memory(
        &mut self,
        shared_memory_id: String,
        free: bool,
    ) -> Result<Metadata, eyre::Error> {
        self.control_channel
            .read_pinned_memory(shared_memory_id, free)
    }

    /// Free a pinned memory pool via the daemon.
    pub fn free_pinned_memory(&mut self, shared_memory_id: String) -> Result<(), eyre::Error> {
        self.control_channel.free_pinned_memory(shared_memory_id)
    }
}

/// Builder for initializing a node with custom connection parameters.
///
/// Created via [`DoraNode::builder()`]. Callers who don't need a custom daemon
/// port should prefer [`DoraNode::init_from_env`] or
/// [`DoraNode::init_from_node_id`]. Setting [`node_id`](Self::node_id) selects
/// the dynamic-node path; otherwise [`build`](Self::build) falls back to
/// [`DoraNode::init_from_env`].
#[derive(Default)]
pub struct DoraNodeBuilder {
    node_id: Option<NodeId>,
    daemon_port: Option<u16>,
}

impl DoraNodeBuilder {
    /// Set the node ID. Presence of a node ID selects the dynamic-node path.
    pub fn node_id(mut self, node_id: NodeId) -> Self {
        self.node_id = Some(node_id);
        self
    }

    /// No-op kept for source compatibility with upstream dora 0.5.x
    /// [`#1591`](https://github.com/dora-rs/dora/pull/1591). Upstream gates the
    /// dynamic-node path on an explicit `.dynamic()` call; here, dynamic mode
    /// is selected by the presence of `node_id`, making the flag redundant.
    /// Kept so that `.node_id(id).dynamic().build()` written against upstream
    /// still compiles.
    #[inline]
    pub fn dynamic(self) -> Self {
        self
    }

    /// Override the daemon port. When unset, the builder honours the
    /// `DORA_DAEMON_LOCAL_LISTEN_PORT` env var and falls back to
    /// `DORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT`.
    pub fn daemon_port(mut self, port: u16) -> Self {
        self.daemon_port = Some(port);
        self
    }

    /// Build and connect the node.
    pub fn build(self) -> NodeResult<(DoraNode, EventStream)> {
        let Some(node_id) = self.node_id else {
            return DoraNode::init_from_env();
        };

        let port = self.daemon_port.unwrap_or_else(|| {
            match std::env::var(DORA_DAEMON_LOCAL_LISTEN_PORT_ENV) {
                Ok(p) => p.parse().unwrap_or_else(|e| {
                    tracing::warn!(
                        "invalid {DORA_DAEMON_LOCAL_LISTEN_PORT_ENV}={p:?}: {e}, using default port"
                    );
                    DORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT
                }),
                Err(_) => DORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT,
            }
        });
        let daemon_address = (LOCALHOST, port).into();

        let mut channel =
            DaemonChannel::new_tcp(daemon_address).context("Could not connect to the daemon")?;
        let clock = Arc::new(uhlc::HLC::default());

        let reply = channel
            .request(&Timestamped {
                inner: DaemonRequest::NodeConfig { node_id },
                timestamp: clock.new_timestamp(),
            })
            .wrap_err("failed to request node config from daemon")?;

        match reply {
            DaemonReply::NodeConfig {
                result: Ok(node_config),
            } => DoraNode::init(node_config),
            DaemonReply::NodeConfig { result: Err(error) } => {
                let capped: String = error.chars().take(512).collect();
                Err(NodeError::Init(format!(
                    "failed to get node config from daemon: {capped}"
                )))
            }
            _ => Err(NodeError::Init("unexpected reply from daemon".into())),
        }
    }
}

/// Runs `teardown` on a dedicated thread, waiting at most `timeout` for it to
/// complete. Returns `true` if the teardown finished in time. Panics in
/// `teardown` are contained and count as completion. If spawning the thread
/// fails, the teardown runs inline without a deadline.
///
/// On timeout the thread keeps running detached: everything moved into the
/// closure (zenoh sockets, SHM segments, the owned tokio runtime) is leaked
/// until process exit. That is acceptable for nodes dropped right before
/// exit; long-lived hosts (e.g. a Python interpreter dropping a node during
/// GC) inherit only the bounded delay instead of a permanent hang.
pub(crate) fn teardown_with_timeout(
    label: &str,
    timeout: Duration,
    teardown: impl FnOnce() + Send + 'static,
) -> bool {
    // The closure is handed over via a channel (instead of being captured by
    // the spawned closure) so that it stays available for the inline
    // fallback when spawning fails.
    let (work_tx, work_rx) = std::sync::mpsc::channel();
    let (done_tx, done_rx) = std::sync::mpsc::channel();
    let thread = std::thread::Builder::new()
        .name(format!("dora-teardown-{label}"))
        .spawn(move || {
            if let Ok(work) = work_rx.recv() {
                let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(work));
            }
            let _ = done_tx.send(());
        });
    match thread {
        Ok(_) => {
            let _ = work_tx.send(teardown);
            done_rx.recv_timeout(timeout).is_ok()
        }
        Err(err) => {
            warn!("failed to spawn {label} teardown thread ({err}); running it inline");
            let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(teardown));
            true
        }
    }
}

impl Drop for DoraNode {
    fn drop(&mut self) {
        // Tear down zenoh before notifying the daemon below, so that
        // daemon-signaled `InputClosed` cannot overtake in-flight zenoh data.
        let publishers = std::mem::take(&mut self.zenoh_publishers);
        let schema_publishers = std::mem::take(&mut self.zenoh_schema_publishers);
        // Per-output readiness counters hold liveliness subscribers on the shared
        // session; drop them before the session (same reasoning as publishers).
        let readiness = std::mem::take(&mut self.zenoh_output_readiness);
        let shm_provider = self.zenoh_shm_provider.take();
        let session = self.zenoh_session.take();
        let runtime = self._owned_runtime.take();
        if session.is_none() && shm_provider.is_none() && publishers.is_empty() {
            // no zenoh state (interactive/testing mode): drop inline
            drop(runtime);
        } else {
            // A wedged zenoh net runtime stalls `Session` close beyond its
            // 10s timeout and `Publisher` undeclare indefinitely, which would
            // hang node shutdown (and with it the daemon, which waits for
            // `InputClosed`). Bound the teardown with a deadline instead.
            let completed = teardown_with_timeout("zenoh", ZENOH_TEARDOWN_TIMEOUT, move || {
                // documented drop order: publishers (data + schema) and readiness
                // subscribers before the session, owned runtime last so async
                // cleanup can still run
                drop(publishers);
                drop(schema_publishers);
                drop(readiness);
                drop(shm_provider);
                drop(session);
                drop(runtime);
            });
            if !completed {
                warn!(
                    "zenoh teardown timed out after {}s; continuing node shutdown",
                    ZENOH_TEARDOWN_TIMEOUT.as_secs()
                );
            }
        }

        // close all outputs first to notify subscribers as early as possible
        if let Err(err) = self
            .control_channel
            .report_closed_outputs(
                std::mem::take(&mut self.node_config.outputs)
                    .into_iter()
                    .collect(),
            )
            .context("failed to close outputs on drop")
        {
            tracing::warn!("{err:?}")
        }

        if let Err(err) = self.control_channel.report_outputs_done() {
            tracing::warn!("{err:?}")
        }
    }
}

/// A data region suitable for sending as an output message.
///
/// `DataSample` implements the [`Deref`](std::ops::Deref) and
/// [`DerefMut`](std::ops::DerefMut) traits to read and write the mapped data.
///
/// The backing storage is either a heap buffer or — for payloads at or above
/// the zero-copy threshold when a zenoh SHM provider is available — a
/// zenoh-shared-memory buffer. Writing into an SHM-backed sample lets the
/// producer construct the message straight in shared memory, so publishing it
/// needs no further copy (the SHM buffer is moved directly into zenoh's `put`).
pub struct DataSample {
    storage: SampleStorage,
}

/// Backing storage for a [`DataSample`]. Kept private so the public API never
/// exposes a zenoh SHM type; callers only ever see the `[u8]` view via
/// `Deref`/`DerefMut`.
enum SampleStorage {
    /// Heap-allocated, 128-byte-aligned buffer (used below the zero-copy
    /// threshold or when no SHM provider is available).
    Heap(AVec<u8, ConstAlign<128>>),
    /// Zenoh shared-memory buffer. The producer writes the payload directly
    /// into it and the buffer is later moved into the zenoh `put` without
    /// copying.
    Shm(zenoh::shm::ZShmMut),
}

impl DataSample {
    /// Consume the sample into a [`FinalizedSample`] ready for transport.
    fn finalize(self) -> FinalizedSample {
        match self.storage {
            SampleStorage::Heap(buffer) => FinalizedSample::Vec(buffer),
            SampleStorage::Shm(sbuf) => FinalizedSample::Shm(sbuf),
        }
    }
}

impl std::ops::Deref for DataSample {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        match &self.storage {
            SampleStorage::Heap(buffer) => buffer,
            SampleStorage::Shm(sbuf) => sbuf.as_ref(),
        }
    }
}

impl std::ops::DerefMut for DataSample {
    fn deref_mut(&mut self) -> &mut Self::Target {
        match &mut self.storage {
            SampleStorage::Heap(buffer) => buffer,
            SampleStorage::Shm(sbuf) => sbuf.as_mut(),
        }
    }
}

impl From<AVec<u8, ConstAlign<128>>> for DataSample {
    fn from(value: AVec<u8, ConstAlign<128>>) -> Self {
        Self {
            storage: SampleStorage::Heap(value),
        }
    }
}

impl std::fmt::Debug for DataSample {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("DataSample")
            .field("len", &self.len())
            .finish_non_exhaustive()
    }
}

/// A finalized output payload ready for transport.
///
/// Kept separate from [`DataMessage`] so SHM buffers stay out of the
/// `Serialize`/`Deserialize` TCP path: the zenoh data plane moves an `Shm`
/// buffer straight into `put` (zero extra copy), while the daemon fallback
/// converts to [`DataMessage::Vec`], copying out of shared memory only when the
/// zenoh path could not deliver.
enum FinalizedSample {
    Vec(AVec<u8, ConstAlign<128>>),
    Shm(zenoh::shm::ZShmMut),
}

impl FinalizedSample {
    fn byte_len(&self) -> usize {
        match self {
            FinalizedSample::Vec(v) => v.len(),
            FinalizedSample::Shm(sbuf) => sbuf.as_ref().len(),
        }
    }

    /// Convert into a TCP-transportable [`DataMessage`]. For the `Shm` arm this
    /// copies the payload out of shared memory into a heap buffer; it runs only
    /// on the daemon fallback (no matching zenoh subscriber / no session).
    fn into_data_message(self) -> DataMessage {
        match self {
            FinalizedSample::Vec(v) => DataMessage::Vec(v),
            FinalizedSample::Shm(sbuf) => {
                let bytes = sbuf.as_ref();
                let mut avec: AVec<u8, ConstAlign<128>> = AVec::__from_elem(128, 0, bytes.len());
                avec.copy_from_slice(bytes);
                DataMessage::Vec(avec)
            }
        }
    }
}

/// Outcome of a zenoh publish attempt.
enum PublishOutcome {
    /// The payload was delivered to zenoh (or, on a rare SHM put error,
    /// consumed and lost — see [`DoraNode::zenoh_publish`]).
    Published,
    /// No matching subscriber, or a transport error before the payload was
    /// consumed. The sample is returned so the caller can fall back to the
    /// daemon path.
    NotPublished(FinalizedSample),
}

/// FNV-1a hash of `bytes` with a fixed seed (cross-process deterministic).
/// Delegates to [`dora_message::metadata::fnv1a`] — the single source of truth
/// shared with the daemon's `dora topic` debug path, so schema hashes match.
pub(crate) fn fnv1a(bytes: &[u8]) -> u64 {
    dora_message::metadata::fnv1a(bytes)
}

/// How often a schema-once output re-sends a full self-describing stream on the
/// data topic. Full streams prime receivers in-band, so this bounds how long a
/// consumer that missed the single `@schema` emission (e.g. a failed zenoh-ext
/// history query) drops schema-less batches: it re-primes at the next refresh
/// instead of losing the input permanently. ~400 B of extra framing per output
/// per interval — negligible.
pub(crate) const SCHEMA_ONCE_REFRESH_INTERVAL: Duration = Duration::from_secs(5);

/// Producer-side schema-once state for one output.
struct SchemaOnceState {
    /// Hash of the schema confirmed published on the `@schema` subtopic.
    published_hash: u64,
    /// When the last full self-describing stream was sent on the data topic.
    last_full_stream: Instant,
}

/// What `publish_schema_once` should do for the current message.
#[derive(Debug)]
enum SchemaOnceDecision {
    /// The schema for this hash is not confirmed published (first message,
    /// schema change, or an earlier `@schema` publish failed): publish it and
    /// send this message as a full self-describing stream. The full stream
    /// decodes standalone and primes receivers in-band — in data-plane order —
    /// so the first message of an output cannot be lost to the express batch
    /// racing ahead of the schema on the separate `@schema` plane, and a failed
    /// schema publish degrades to "full stream every message" (decodable)
    /// instead of "hash-tagged but undecodable" (dora-rs/dora#2366 review).
    PublishSchemaAndSendFullStream,
    /// The periodic full-stream refresh is due (see
    /// [`SCHEMA_ONCE_REFRESH_INTERVAL`]).
    SendFullStreamRefresh,
    /// Schema confirmed published and fresh: send only the schema-less batch,
    /// tagged with the schema hash.
    SendSchemaLessBatch,
}

fn schema_once_decision(
    state: Option<&SchemaOnceState>,
    hash: u64,
    now: Instant,
) -> SchemaOnceDecision {
    match state {
        Some(state) if state.published_hash == hash => {
            if now.duration_since(state.last_full_stream) >= SCHEMA_ONCE_REFRESH_INTERVAL {
                SchemaOnceDecision::SendFullStreamRefresh
            } else {
                SchemaOnceDecision::SendSchemaLessBatch
            }
        }
        _ => SchemaOnceDecision::PublishSchemaAndSendFullStream,
    }
}

/// Publish the Arrow IPC schema for `output_id` on its `@schema` subtopic when
/// it changes, and return the attachment metadata (carrying the schema hash)
/// for the schema-less batch the caller sends on the data topic. Returns `None`
/// when the caller must send the full self-describing stream instead: on the
/// message that (re)publishes the schema, when the `@schema` publish failed,
/// for the periodic full-stream refresh, or if `full_stream` is not a parseable
/// IPC stream (see [`SchemaOnceDecision`]).
///
/// Takes the maps by `&mut` (not `&mut self`) so it can run while an immutable
/// borrow of `self.zenoh_publishers` (the data publisher) is live.
#[allow(clippy::too_many_arguments)]
fn publish_schema_once(
    schema_publishers: &mut HashMap<DataId, zenoh_ext::AdvancedPublisher<'static>>,
    schema_state: &mut HashMap<DataId, SchemaOnceState>,
    session: &zenoh::Session,
    dataflow_id: DataflowId,
    node_id: &NodeId,
    output_id: &DataId,
    full_stream: &[u8],
    base_metadata: &Metadata,
) -> Option<Vec<u8>> {
    let (hash, schema_bytes) = arrow_utils::ipc_encode::schema_block_and_hash(full_stream)?;

    let now = Instant::now();
    let decision = schema_once_decision(schema_state.get(output_id), hash, now);
    tracing::debug!(output = %output_id, decision = ?decision, "schema-once decision");

    match decision {
        SchemaOnceDecision::PublishSchemaAndSendFullStream => {
            if let Some(publisher) =
                schema_publisher(schema_publishers, session, dataflow_id, node_id, output_id)
            {
                use zenoh::Wait;
                match publisher.put(schema_bytes).wait() {
                    // Record the hash only on a successful publish, so a failed
                    // emission is retried on the next message rather than
                    // silently skipped.
                    Ok(()) => {
                        tracing::debug!(output = %output_id, hash, "schema published on @schema subtopic");
                        schema_state.insert(
                            output_id.clone(),
                            SchemaOnceState {
                                published_hash: hash,
                                last_full_stream: now,
                            },
                        );
                    }
                    Err(e) => {
                        tracing::warn!(output = %output_id, "failed to publish schema on @schema subtopic ({e})");
                    }
                }
            }
            None
        }
        SchemaOnceDecision::SendFullStreamRefresh => {
            tracing::debug!(output = %output_id, hash, "sending full-stream refresh");
            if let Some(state) = schema_state.get_mut(output_id) {
                state.last_full_stream = now;
            }
            None
        }
        SchemaOnceDecision::SendSchemaLessBatch => {
            tracing::debug!(output = %output_id, hash, "sending schema-less batch with SCHEMA_HASH");
            // Every batch carries the schema hash so the receiver can match it
            // to the primed decoder (and detect a schema change).
            let mut metadata = base_metadata.clone();
            metadata
                .parameters
                .insert(SCHEMA_HASH.to_string(), Parameter::Integer(hash as i64));
            bincode::serialize(&metadata).ok()
        }
    }
}

/// Get or lazily declare the schema `AdvancedPublisher` for `output_id` on its
/// `@schema` subtopic. The cache (depth 1) retains the last schema so a
/// late-joining subscriber's history query can fetch it; `publisher_detection`
/// lets a subscriber that started first discover this publisher and query its
/// cache. `CongestionControl::Block` keeps the single live schema emission from
/// being dropped under congestion.
fn schema_publisher<'a>(
    schema_publishers: &'a mut HashMap<DataId, zenoh_ext::AdvancedPublisher<'static>>,
    session: &zenoh::Session,
    dataflow_id: DataflowId,
    node_id: &NodeId,
    output_id: &DataId,
) -> Option<&'a zenoh_ext::AdvancedPublisher<'static>> {
    if !schema_publishers.contains_key(output_id) {
        use zenoh::Wait;
        use zenoh::qos::CongestionControl;
        use zenoh_ext::{AdvancedPublisherBuilderExt, CacheConfig, MissDetectionConfig};

        let topic = dora_core::topics::zenoh_output_schema_topic(dataflow_id, node_id, output_id);
        let key = zenoh::key_expr::KeyExpr::new(topic).ok()?.into_owned();
        let publisher = match session
            .declare_publisher(key)
            .congestion_control(CongestionControl::Block)
            // `sample_miss_detection` selects SequenceNumber sequencing instead of
            // the cache's default Timestamp sequencing, so the schema publisher
            // doesn't require session-wide timestamping (which would otherwise add
            // an HLC timestamp to every data-plane message too). Its default
            // config adds no heartbeat, so there's no extra periodic traffic.
            .sample_miss_detection(MissDetectionConfig::default())
            .cache(CacheConfig::default())
            .publisher_detection()
            .wait()
        {
            Ok(p) => p,
            Err(e) => {
                tracing::warn!(output = %output_id, "failed to declare schema publisher ({e})");
                return None;
            }
        };
        schema_publishers.insert(output_id.clone(), publisher);
    }
    schema_publishers.get(output_id)
}

pub(crate) use dora_message::metadata::carries_pattern_correlation;

/// Whether the schema-once optimization may be applied to a data-plane message.
///
/// Eligible only when the payload is below the zero-copy threshold *and* the
/// output does not interleave multiple Arrow schemas. Service/action
/// request-reply messages (`request_id`/`goal_id`/`goal_status`) multiplex
/// response schemas per request and must travel as full self-describing streams
/// so each decodes standalone; streaming chunks share one schema and stay
/// eligible. See the rationale at the call site in `zenoh_publish`.
fn schema_once_eligible(
    payload_len: usize,
    zero_copy_threshold: usize,
    params: &MetadataParameters,
) -> bool {
    payload_len < zero_copy_threshold && !carries_pattern_correlation(params)
}

/// Init Opentelemetry Tracing
///
/// This requires a tokio runtime spawning this function to be functional
#[cfg(feature = "tracing")]
pub fn init_tracing(
    node_id: &NodeId,
    dataflow_id: &DataflowId,
) -> NodeResult<Arc<Mutex<Option<OtelGuard>>>> {
    let node_id_str = node_id.to_string();
    let guard: Arc<Mutex<Option<OtelGuard>>> = Arc::new(Mutex::new(None));
    let clone = guard.clone();
    let tracing_monitor = async move {
        let mut builder = TracingBuilder::new(node_id_str.clone());
        // Only enable OTLP if environment variable is set
        if std::env::var("DORA_OTLP_ENDPOINT").is_ok()
            || std::env::var("DORA_JAEGER_TRACING").is_ok()
        {
            match builder.with_otlp_tracing() {
                Ok(b) => {
                    builder = b.with_stdout("info", true);
                    if let Ok(mut guard) = clone.lock() {
                        *guard = builder.guard.take();
                    }
                }
                Err(e) => {
                    eprintln!("warning: failed to set up OTLP tracing: {e:?}");
                    // Rebuild without OTLP — with_otlp_tracing consumed builder
                    builder = TracingBuilder::new(node_id_str).with_stdout("info", true);
                }
            }
        } else {
            builder = builder.with_stdout("info", true);
        }

        if let Err(e) = builder.build() {
            eprintln!("warning: failed to set up tracing subscriber: {e:?}");
        }
    };

    let rt = Handle::try_current().context("failed to get tokio runtime handle")?;
    rt.spawn(tracing_monitor);

    // dataflow_id is only used when metrics feature is enabled
    let _ = &dataflow_id;

    // Only start the OTLP metrics exporter when an endpoint is configured.
    // The exporter schedules via `tokio::time::interval` and would otherwise
    // panic on callers whose runtime lacks the time driver, and would also
    // attempt to connect to `localhost:4317` on every node startup. Mirrors
    // the gating applied to tracing above.
    #[cfg(feature = "metrics")]
    if std::env::var("DORA_OTLP_ENDPOINT").is_ok() {
        let id = format!("{dataflow_id}/{node_id}");
        let monitor_task = async move {
            use dora_metrics::run_metrics_monitor;

            if let Err(e) = run_metrics_monitor(id.clone())
                .await
                .wrap_err("metrics monitor exited unexpectedly")
            {
                warn!("metrics monitor failed: {:#?}", e);
            }
        };
        let rt = Handle::try_current().context("failed to get tokio runtime handle")?;
        rt.spawn(monitor_task);
    }
    Ok(guard)
}

/// Builder for streaming segment metadata.
///
/// Manages session/segment IDs and auto-incrementing sequence numbers
/// for real-time streaming patterns (voice, video, sensor streams).
pub struct StreamSegment {
    session_id: String,
    segment_id: i64,
    seq: i64,
}

impl StreamSegment {
    /// Start a new session with a generated session ID and segment 0.
    pub fn new() -> Self {
        Self {
            session_id: DoraNode::new_request_id(),
            segment_id: 0,
            seq: 0,
        }
    }

    /// Start a new session with an explicit session ID.
    pub fn with_session_id(session_id: String) -> Self {
        Self {
            session_id,
            segment_id: 0,
            seq: 0,
        }
    }

    /// Advance to a new segment (resets seq to 0). Returns the new segment_id.
    pub fn next_segment(&mut self) -> i64 {
        self.segment_id += 1;
        self.seq = 0;
        self.segment_id
    }

    /// Build metadata parameters for a chunk. Auto-increments seq.
    pub fn chunk(&mut self, fin: bool) -> MetadataParameters {
        let mut params = MetadataParameters::new();
        params.insert(
            SESSION_ID.into(),
            Parameter::String(self.session_id.clone()),
        );
        params.insert(SEGMENT_ID.into(), Parameter::Integer(self.segment_id));
        params.insert(SEQ.into(), Parameter::Integer(self.seq));
        params.insert(FIN.into(), Parameter::Bool(fin));
        self.seq += 1;
        params
    }

    /// Build metadata for a flush message (new segment, discards older queued data).
    ///
    /// Advances to a new segment, then emits a chunk with `flush=true` and
    /// `fin=false`. The prior segment ends without a `fin=true` signal -- this
    /// is intentional for interruption semantics (the old data is being
    /// discarded, not completed).
    ///
    /// **Note**: flush discards *all* queued messages on the receiver's input
    /// regardless of `session_id`. Do not multiplex independent sessions on a
    /// single `DataId` when using flush.
    pub fn flush(&mut self) -> MetadataParameters {
        self.next_segment();
        let mut params = self.chunk(false);
        params.insert(FLUSH.into(), Parameter::Bool(true));
        params
    }

    /// Returns the session ID.
    pub fn session_id(&self) -> &str {
        &self.session_id
    }

    /// Returns the current segment ID.
    pub fn segment_id(&self) -> i64 {
        self.segment_id
    }

    /// Returns the sequence number that will be used by the next `chunk()` call.
    pub fn seq(&self) -> i64 {
        self.seq
    }
}

impl Default for StreamSegment {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::integration_testing::{
        IntegrationTestInput, TestingInput, TestingOptions, TestingOutput,
        integration_testing_format::{IncomingEvent, TimedIncomingEvent},
    };
    use arrow::array::NullArray;

    #[test]
    fn new_request_id_returns_valid_uuid() {
        let id = DoraNode::new_request_id();
        uuid::Uuid::parse_str(&id).expect("should be valid UUID");
    }

    #[test]
    fn new_request_id_is_unique() {
        let ids: Vec<String> = (0..100).map(|_| DoraNode::new_request_id()).collect();
        let unique: std::collections::HashSet<_> = ids.iter().collect();
        assert_eq!(ids.len(), unique.len(), "all IDs should be unique");
    }

    #[test]
    fn new_goal_id_returns_valid_uuid() {
        let id = DoraNode::new_goal_id();
        uuid::Uuid::parse_str(&id).expect("should be valid UUID");
    }

    /// `DoraNode::timestamp()` must read from the SAME HLC the node
    /// uses to stamp outgoing messages. If a refactor accidentally
    /// gives `timestamp()` its own clock, the latency-measurement use
    /// case in the docstring silently breaks (subtracting against an
    /// `event.metadata.timestamp` from the data plane would mix two
    /// unrelated HLCs). Guard by asserting two calls share an HLC ID
    /// and that the second reads strictly later than the first.
    ///
    /// The strict `t2 > t1` assertion holds by HLC construction: if
    /// the wall clock advanced between calls, the physical component
    /// strictly increases; if not, the logical counter bumps. The
    /// lexicographic ordering on `uhlc::Timestamp` puts `t2` strictly
    /// after `t1` in either case, so this assertion does not flake on
    /// fast machines whose OS clock rounds both calls to the same tick.
    #[test]
    fn timestamp_uses_node_clock_and_is_monotonic() {
        let (node, events, _rx) = test_node();
        let t1 = node.timestamp();
        let t2 = node.timestamp();
        assert_eq!(
            t1.get_id(),
            t2.get_id(),
            "two timestamp() calls must come from the same HLC instance",
        );
        assert!(
            t2 > t1,
            "HLC timestamps must be strictly monotonic: {t1:?} >= {t2:?}"
        );
        drop(node);
        drop(events);
    }

    /// Helper: create a minimal test node with a channel output.
    fn test_node() -> (
        DoraNode,
        crate::EventStream,
        flume::Receiver<serde_json::Map<String, serde_json::Value>>,
    ) {
        let events = vec![TimedIncomingEvent {
            time_offset_secs: 0.1,
            event: IncomingEvent::Stop,
        }];
        let inputs = TestingInput::Input(IntegrationTestInput::new(
            "test-node".parse().unwrap(),
            events,
        ));
        let (tx, rx) = flume::unbounded();
        let outputs = TestingOutput::ToChannel(tx);
        let options = TestingOptions {
            skip_output_time_offsets: true,
        };
        let (node, event_stream) = DoraNode::init_testing(inputs, outputs, options).unwrap();
        (node, event_stream, rx)
    }

    #[test]
    fn send_service_request_returns_valid_id_and_sends_output() {
        let (mut node, events, rx) = test_node();

        let request_id = node
            .send_service_request("request".into(), Default::default(), NullArray::new(0))
            .unwrap();

        // Returned ID should be a valid UUID
        uuid::Uuid::parse_str(&request_id).expect("returned request_id should be valid UUID");

        // Output should have been sent to the channel
        drop(node);
        drop(events);
        let outputs: Vec<_> = rx.try_iter().collect();
        assert_eq!(outputs.len(), 1);
        assert_eq!(outputs[0]["id"], "request");
    }

    #[test]
    fn send_service_request_returns_unique_ids() {
        let (mut node, events, _rx) = test_node();

        let id1 = node
            .send_service_request("out".into(), Default::default(), NullArray::new(0))
            .unwrap();
        let id2 = node
            .send_service_request("out".into(), Default::default(), NullArray::new(0))
            .unwrap();

        assert_ne!(id1, id2, "successive request IDs should differ");

        drop(node);
        drop(events);
    }

    #[test]
    fn send_service_response_sends_output() {
        let (mut node, events, rx) = test_node();

        // Simulate passing through a request_id from the incoming request
        let mut params = MetadataParameters::default();
        params.insert(
            dora_message::metadata::REQUEST_ID.to_string(),
            dora_message::metadata::Parameter::String("test-req-id".into()),
        );
        node.send_service_response("response".into(), params, NullArray::new(0))
            .unwrap();

        drop(node);
        drop(events);
        let outputs: Vec<_> = rx.try_iter().collect();
        assert_eq!(outputs.len(), 1);
        assert_eq!(outputs[0]["id"], "response");
    }

    /// `send_output_bytes` must reject a `data_len` that disagrees with
    /// `data.len()` with a clear error instead of panicking inside
    /// `copy_from_slice` deep in `send_output_raw`.
    #[test]
    fn send_output_bytes_rejects_len_mismatch() {
        let (mut node, events, _rx) = test_node();

        let result = node.send_output_bytes("out".into(), Default::default(), 8, &[1, 2, 3, 4]);

        let err = result.expect_err("mismatched data_len must error, not panic");
        assert!(
            err.to_string().contains("does not match"),
            "unexpected error message: {err}"
        );

        drop(node);
        drop(events);
    }

    /// A heap-backed `DataSample` is writable through `DerefMut`, readable
    /// through `Deref`, and `finalize().into_data_message()` preserves the bytes
    /// as the `DataMessage::Vec` daemon-path payload. (The SHM-backed arm needs
    /// a live zenoh provider and is covered by the copy-count harness/smoke.)
    #[test]
    fn data_sample_heap_roundtrip() {
        let avec: AVec<u8, ConstAlign<128>> = AVec::__from_elem(128, 0, 8);
        let mut sample: DataSample = avec.into();
        sample.copy_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8]);

        assert_eq!(&sample[..], &[1, 2, 3, 4, 5, 6, 7, 8]);
        assert_eq!(sample.len(), 8);

        match sample.finalize().into_data_message() {
            DataMessage::Vec(v) => assert_eq!(v.as_slice(), &[1, 2, 3, 4, 5, 6, 7, 8]),
        }
    }

    /// End-to-end wire contract: a few representative arrays IPC-encoded (fast
    /// path) into a sample and decoded back via `decode_arrow_ipc_zero_copy`
    /// must equal the input. This is the send->receive round-trip the data
    /// plane relies on (zenoh can't be smoke-tested here, so this stands in).
    #[test]
    fn send_output_ipc_roundtrip() {
        use crate::arrow_utils::decode_arrow_ipc_zero_copy;
        use crate::arrow_utils::ipc_encode::{encode_ipc_into, ipc_fast_path_len};
        use arrow::array::{ArrayRef, Float32Array, StringArray, StructArray, UInt64Array};
        use arrow_schema::{DataType, Field};
        use std::ptr::NonNull;

        fn roundtrip(data: ArrayData) {
            let len = ipc_fast_path_len(&data).expect("array should be fast-path eligible");
            let mut buf: AVec<u8, ConstAlign<128>> = AVec::__from_elem(128, 0, len);
            encode_ipc_into(&data, &mut buf).expect("fast-path IPC encode");

            // Wrap the aligned sample as an Arrow Buffer (no copy), mirroring the
            // receive path, then decode.
            let ptr = NonNull::new(buf.as_ptr() as *mut u8).unwrap();
            let blen = buf.len();
            // SAFETY: ptr/len describe `buf`; the Arc keeps it alive.
            let buffer =
                unsafe { arrow::buffer::Buffer::from_custom_allocation(ptr, blen, Arc::new(buf)) };
            let decoded = decode_arrow_ipc_zero_copy(buffer).expect("zero-copy IPC decode");
            assert_eq!(
                data, decoded,
                "IPC send->receive round-trip must preserve the array"
            );
        }

        roundtrip(Float32Array::from(vec![1.0, 2.5, -3.0, 4.0]).into_data());
        roundtrip(UInt64Array::from(vec![Some(1), None, Some(3)]).into_data());
        roundtrip(StringArray::from(vec![Some("hello"), None, Some("world")]).into_data());
        roundtrip(
            StructArray::from(vec![
                (
                    Arc::new(Field::new("v", DataType::UInt64, true)),
                    Arc::new(UInt64Array::from(vec![Some(1), None, Some(3)])) as ArrayRef,
                ),
                (
                    Arc::new(Field::new("s", DataType::Utf8, true)),
                    Arc::new(StringArray::from(vec![Some("a"), Some("bb"), None])) as ArrayRef,
                ),
            ])
            .into_data(),
        );
    }

    /// `close_outputs` must be atomic: if any id in the batch is unknown, the
    /// call fails *without* removing the valid ids from the local output set.
    /// Otherwise the daemon (never notified, because `report_closed_outputs` is
    /// skipped on error) and the node disagree about which outputs are open, and
    /// the node would silently drop subsequent sends to a still-open output.
    #[test]
    fn close_outputs_is_atomic_on_unknown_id() {
        let (mut node, events, _rx) = test_node();
        let valid: DataId = "valid".into();
        node.node_config.outputs.insert(valid.clone());

        let result = node.close_outputs(vec![valid.clone(), "unknown".into()]);

        assert!(
            result.is_err(),
            "closing a batch containing an unknown output must fail"
        );
        assert!(
            node.node_config.outputs.contains(&valid),
            "a failed close_outputs must not remove the valid output from local state"
        );

        drop(node);
        drop(events);
    }

    // ---- dora-rs/adora#150: pattern polymorphism exemption ----

    #[test]
    fn carries_pattern_correlation_detects_request_id() {
        let mut params = MetadataParameters::default();
        params.insert(
            dora_message::metadata::REQUEST_ID.to_string(),
            dora_message::metadata::Parameter::String("req-1".into()),
        );
        assert!(carries_pattern_correlation(&params));
    }

    #[test]
    fn carries_pattern_correlation_detects_goal_id() {
        let mut params = MetadataParameters::default();
        params.insert(
            dora_message::metadata::GOAL_ID.to_string(),
            dora_message::metadata::Parameter::String("goal-1".into()),
        );
        assert!(carries_pattern_correlation(&params));
    }

    #[test]
    fn carries_pattern_correlation_detects_goal_status() {
        let mut params = MetadataParameters::default();
        params.insert(
            dora_message::metadata::GOAL_STATUS.to_string(),
            dora_message::metadata::Parameter::String("succeeded".into()),
        );
        assert!(carries_pattern_correlation(&params));
    }

    #[test]
    fn carries_pattern_correlation_empty_is_not_a_pattern() {
        let params = MetadataParameters::default();
        assert!(!carries_pattern_correlation(&params));
    }

    #[test]
    fn carries_pattern_correlation_ignores_non_pattern_keys() {
        let mut params = MetadataParameters::default();
        params.insert(
            "custom_key".to_string(),
            dora_message::metadata::Parameter::String("value".into()),
        );
        assert!(!carries_pattern_correlation(&params));
    }

    #[test]
    fn schema_once_excludes_pattern_correlation_outputs() {
        // Regression (dora-rs/dora#2366 review): a small service/action
        // request-reply message — which multiplexes response schemas per request
        // — must NOT use schema-once, or a schema-less batch could reach a
        // consumer primed for a different schema and be silently dropped. It must
        // travel as a full self-describing stream instead.
        const THRESHOLD: usize = 4096;

        let plain = MetadataParameters::default();
        assert!(
            schema_once_eligible(100, THRESHOLD, &plain),
            "small message on a stable-schema output is eligible"
        );
        assert!(
            !schema_once_eligible(THRESHOLD, THRESHOLD, &plain),
            "a message at/above the threshold is not eligible (goes via SHM/full stream)"
        );

        for key in [
            dora_message::metadata::REQUEST_ID,
            dora_message::metadata::GOAL_ID,
            dora_message::metadata::GOAL_STATUS,
        ] {
            let mut params = MetadataParameters::default();
            params.insert(
                key.to_string(),
                dora_message::metadata::Parameter::String("x".into()),
            );
            assert!(
                !schema_once_eligible(100, THRESHOLD, &params),
                "small pattern-correlation message ({key}) must bypass schema-once"
            );
        }

        // Streaming is deliberately NOT excluded: every chunk of a stream shares
        // one schema, so streaming stays the high-rate beneficiary of
        // schema-once. Locking this in guards against a well-meaning "also
        // exclude streaming" change that would defeat the optimization.
        let mut stream = MetadataParameters::default();
        stream.insert(
            dora_message::metadata::SESSION_ID.to_string(),
            dora_message::metadata::Parameter::String("s1".into()),
        );
        stream.insert(
            dora_message::metadata::SEGMENT_ID.to_string(),
            dora_message::metadata::Parameter::Integer(0),
        );
        assert!(
            schema_once_eligible(100, THRESHOLD, &stream),
            "small streaming chunk (stable schema) stays eligible for schema-once"
        );
    }

    #[test]
    fn schema_once_decision_covers_publish_refresh_and_schema_less() {
        let start = Instant::now();
        let later = start + SCHEMA_ONCE_REFRESH_INTERVAL;
        let state = SchemaOnceState {
            published_hash: 7,
            last_full_stream: start,
        };

        // No state yet (first message of this output) → publish the schema and
        // send THIS message as a full stream: it decodes standalone and primes
        // receivers in-band, so the first message can never be lost to the
        // batch racing ahead of the schema on the separate `@schema` plane
        // (dora-rs/dora#2366 review).
        assert!(matches!(
            schema_once_decision(None, 7, start),
            SchemaOnceDecision::PublishSchemaAndSendFullStream
        ));
        // Schema changed (or an earlier `@schema` publish failed, which leaves
        // the recorded hash stale) → same: publish + full stream.
        assert!(matches!(
            schema_once_decision(Some(&state), 8, start),
            SchemaOnceDecision::PublishSchemaAndSendFullStream
        ));
        // Schema confirmed published and refresh not due → schema-less batch.
        assert!(matches!(
            schema_once_decision(Some(&state), 7, start),
            SchemaOnceDecision::SendSchemaLessBatch
        ));
        // Refresh due → send a full stream so any consumer that missed the
        // single `@schema` emission re-primes in-band within the interval
        // instead of losing the input permanently.
        assert!(matches!(
            schema_once_decision(Some(&state), 7, later),
            SchemaOnceDecision::SendFullStreamRefresh
        ));
    }

    #[test]
    fn stream_segment_new_generates_valid_session_id() {
        let seg = StreamSegment::new();
        uuid::Uuid::parse_str(seg.session_id()).expect("session_id should be valid UUID");
        assert_eq!(seg.segment_id(), 0);
    }

    #[test]
    fn stream_segment_with_session_id() {
        let seg = StreamSegment::with_session_id("my-session".into());
        assert_eq!(seg.session_id(), "my-session");
        assert_eq!(seg.segment_id(), 0);
        assert_eq!(seg.seq(), 0);
    }

    #[test]
    fn stream_segment_seq_accessor_tracks_next_seq() {
        let mut seg = StreamSegment::with_session_id("s1".into());
        assert_eq!(seg.seq(), 0);
        seg.chunk(false);
        assert_eq!(seg.seq(), 1);
        seg.chunk(false);
        assert_eq!(seg.seq(), 2);
        seg.next_segment();
        assert_eq!(seg.seq(), 0);
    }

    #[test]
    fn stream_segment_chunk_auto_increments_seq() {
        let mut seg = StreamSegment::with_session_id("s1".into());
        let p0 = seg.chunk(false);
        let p1 = seg.chunk(false);
        let p2 = seg.chunk(true);

        assert_eq!(p0.get(SEQ), Some(&Parameter::Integer(0)));
        assert_eq!(p1.get(SEQ), Some(&Parameter::Integer(1)));
        assert_eq!(p2.get(SEQ), Some(&Parameter::Integer(2)));
        assert_eq!(p0.get(FIN), Some(&Parameter::Bool(false)));
        assert_eq!(p2.get(FIN), Some(&Parameter::Bool(true)));
        assert_eq!(p0.get(SESSION_ID), Some(&Parameter::String("s1".into())));
        assert_eq!(p0.get(SEGMENT_ID), Some(&Parameter::Integer(0)));
    }

    #[test]
    fn stream_segment_next_segment_resets_seq() {
        let mut seg = StreamSegment::with_session_id("s1".into());
        seg.chunk(false); // seq=0
        seg.chunk(false); // seq=1
        let new_id = seg.next_segment();
        assert_eq!(new_id, 1);
        assert_eq!(seg.segment_id(), 1);

        let p = seg.chunk(false);
        assert_eq!(p.get(SEQ), Some(&Parameter::Integer(0)));
        assert_eq!(p.get(SEGMENT_ID), Some(&Parameter::Integer(1)));
    }

    #[test]
    fn stream_segment_flush_advances_segment_and_sets_flush() {
        let mut seg = StreamSegment::with_session_id("s1".into());
        seg.chunk(false);
        let p = seg.flush();
        assert_eq!(seg.segment_id(), 1);
        assert_eq!(p.get(FLUSH), Some(&Parameter::Bool(true)));
        assert_eq!(p.get(SEGMENT_ID), Some(&Parameter::Integer(1)));
        // flush resets seq, then chunk increments it to 1
        assert_eq!(p.get(SEQ), Some(&Parameter::Integer(0)));
    }

    #[test]
    fn send_stream_chunk_sends_output() {
        let (mut node, events, rx) = test_node();
        let mut seg = StreamSegment::with_session_id("s1".into());

        node.send_stream_chunk("audio".into(), &mut seg, false, NullArray::new(0))
            .unwrap();

        drop(node);
        drop(events);
        let outputs: Vec<_> = rx.try_iter().collect();
        assert_eq!(outputs.len(), 1);
        assert_eq!(outputs[0]["id"], "audio");
    }

    #[test]
    fn teardown_with_timeout_completes_fast_closure() {
        let start = Instant::now();
        let completed = teardown_with_timeout("fast", Duration::from_secs(5), || {});
        assert!(completed, "fast teardown should report completion");
        assert!(
            start.elapsed() < Duration::from_secs(5),
            "fast teardown should not wait for the full timeout"
        );
    }

    #[test]
    fn teardown_with_timeout_gives_up_on_wedged_closure() {
        let start = Instant::now();
        let completed = teardown_with_timeout("wedged", Duration::from_millis(300), || {
            std::thread::sleep(Duration::from_secs(60))
        });
        let elapsed = start.elapsed();
        assert!(!completed, "wedged teardown should report a timeout");
        assert!(
            elapsed >= Duration::from_millis(300),
            "should wait the full deadline, returned after {elapsed:?}"
        );
        assert!(
            elapsed < Duration::from_secs(5),
            "should give up shortly after the deadline, took {elapsed:?}"
        );
    }

    #[test]
    fn teardown_with_timeout_contains_panics() {
        let completed = teardown_with_timeout("panicking", Duration::from_secs(5), || {
            panic!("teardown panicked")
        });
        assert!(completed, "panicking teardown still counts as completed");
    }
}
