use crate::{
    DaemonCommunicationWrapper, EventStream, NodeError, NodeResult,
    daemon_connection::{DaemonChannel, IntegrationTestingEvents},
    integration_testing::{
        TestingCommunication, TestingInput, TestingOptions, TestingOutput,
        take_testing_communication,
    },
};

use self::{
    arrow_utils::{
        compute_schema_hash, copy_array_into_sample, encode_arrow_ipc, required_data_size,
    },
    control_channel::ControlChannel,
    drop_stream::DropStream,
};
use adora_core::{
    config::{DataId, NodeId, NodeRunConfig},
    descriptor::Descriptor,
    metadata::ArrowTypeInfoExt,
    topics::{
        ADORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT, ADORA_DAEMON_LOCAL_LISTEN_PORT_ENV, LOCALHOST,
    },
    types::TypeRegistry,
    uhlc,
};
use adora_message::{
    DataflowId,
    daemon_to_node::{DaemonCommunication, DaemonReply, NodeConfig},
    descriptor::OutputFraming,
    metadata::{
        ArrowTypeInfo, FIN, FLUSH, FRAMING, FRAMING_ARROW_IPC, Metadata, MetadataParameters,
        Parameter, SEGMENT_ID, SEQ, SESSION_ID,
    },
    node_to_daemon::{DaemonRequest, DataMessage, DropToken, Timestamped},
};
use aligned_vec::{AVec, ConstAlign};
use arrow::array::Array;
use colored::Colorize;
use eyre::{WrapErr, bail};
use is_terminal::IsTerminal;
use shared_memory_extended::{Shmem, ShmemConf};

#[cfg(feature = "tracing")]
use std::sync::Mutex;
use std::{
    collections::{BTreeSet, HashMap, VecDeque},
    ops::{Deref, DerefMut},
    path::PathBuf,
    sync::Arc,
    time::Duration,
};
#[cfg(feature = "tracing")]
use tokio::runtime::Handle;

#[cfg(feature = "tracing")]
use adora_tracing::{OtelGuard, TracingBuilder};
use tracing::{info, warn};

pub mod arrow_utils;
mod control_channel;
mod drop_stream;

/// Runtime type checking mode, controlled by `ADORA_RUNTIME_TYPE_CHECK` env var.
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
        match std::env::var("ADORA_RUNTIME_TYPE_CHECK").as_deref() {
            Ok("error") => Self::Error,
            Ok("1" | "warn" | "true") => Self::Warn,
            Ok("") | Err(_) => Self::Off,
            Ok(other) => {
                tracing::warn!(
                    "unknown ADORA_RUNTIME_TYPE_CHECK value \"{other}\", \
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

/// Allows sending outputs and retrieving node information.
///
/// The main purpose of this struct is to send outputs via Adora. There are also functions available
/// for retrieving the node configuration.
pub struct AdoraNode {
    id: NodeId,
    dataflow_id: DataflowId,
    node_config: NodeRunConfig,
    control_channel: ControlChannel,
    clock: Arc<uhlc::HLC>,

    sent_out_shared_memory: HashMap<DropToken, ShmemHandle>,
    drop_stream: DropStream,
    cache: VecDeque<ShmemHandle>,

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
    /// Threshold for using zenoh SHM vs inline bytes (default 4096).
    zenoh_zero_copy_threshold: usize,

    dataflow_descriptor: serde_yaml::Result<Descriptor>,
    warned_unknown_output: BTreeSet<DataId>,
    interactive: bool,
    restart_count: u32,

    /// Runtime type checking state. `None` when off (zero overhead).
    /// When `Some`, holds the mode (Warn/Error) and a map of output DataId -> expected Arrow DataType.
    runtime_type_checks: Option<(RuntimeTypeCheck, HashMap<DataId, arrow_schema::DataType>)>,
}

impl AdoraNode {
    /// Initiate a node from environment variables set by the Adora daemon or fall back to
    /// interactive mode.
    ///
    /// This is the recommended initialization function for Adora nodes, which are spawned by
    /// Adora daemon instances. The daemon will set a `ADORA_NODE_CONFIG` environment variable to
    /// configure the node.
    ///
    /// When the node is started manually without the `ADORA_NODE_CONFIG` environment variable set,
    /// the initialization will fall back to [`init_interactive`](Self::init_interactive) if `stdin`
    /// is a terminal (detected through
    /// [`isatty`](https://www.man7.org/linux/man-pages/man3/isatty.3.html)).
    ///
    /// If the `ADORA_NODE_CONFIG` environment variable is not set and `ADORA_TEST_WITH_INPUTS` is
    /// set, the node will be initialized in integration test mode. See the
    /// [integration testing](crate::integration_testing) module for details.
    ///
    /// This function will also initialize the node in integration test mode when the
    /// [`setup_integration_testing`](crate::integration_testing::setup_integration_testing)
    /// function was called before. This takes precedence over all environment variables.
    ///
    /// ```no_run
    /// use adora_node_api::AdoraNode;
    ///
    /// let (mut node, mut events) = AdoraNode::init_from_env().expect("Could not init node.");
    /// ```
    pub fn init_from_env() -> NodeResult<(Self, EventStream)> {
        Self::init_from_env_inner(true)
    }

    /// Initialize the node from environment variables set by the Adora daemon; error if not set.
    ///
    /// This function behaves the same as [`init_from_env`](Self::init_from_env), but it does _not_
    /// fall back to [`init_interactive`](Self::init_interactive). Instead, an error is returned
    /// when the `ADORA_NODE_CONFIG` environment variable is missing.
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

        // normal execution (started by adora daemon)
        match std::env::var("ADORA_NODE_CONFIG") {
            Ok(raw) => {
                let node_config: NodeConfig =
                    serde_yaml::from_str(&raw).context("failed to deserialize node config")?;
                return Self::init(node_config);
            }
            Err(std::env::VarError::NotUnicode(_)) => {
                return Err(NodeError::Init(
                    "ADORA_NODE_CONFIG env variable is not valid unicode".into(),
                ));
            }
            Err(std::env::VarError::NotPresent) => {} // continue trying other init methods
        };

        // node integration test mode
        match std::env::var("ADORA_TEST_WITH_INPUTS") {
            Ok(raw) => {
                let input_file = PathBuf::from(raw);
                let output_file = match std::env::var("ADORA_TEST_WRITE_OUTPUTS_TO") {
                    Ok(raw) => PathBuf::from(raw),
                    Err(std::env::VarError::NotUnicode(_)) => {
                        return Err(NodeError::Init(
                            "ADORA_TEST_WRITE_OUTPUTS_TO env variable is not valid unicode".into(),
                        ));
                    }
                    Err(std::env::VarError::NotPresent) => {
                        input_file.with_file_name("outputs.jsonl")
                    }
                };
                let skip_output_time_offsets =
                    std::env::var_os("ADORA_TEST_NO_OUTPUT_TIME_OFFSET").is_some();

                let input = TestingInput::FromJsonFile(input_file);
                let output = TestingOutput::ToFile(output_file);
                let options = TestingOptions {
                    skip_output_time_offsets,
                };

                return Self::init_testing(input, output, options);
            }
            Err(std::env::VarError::NotUnicode(_)) => {
                return Err(NodeError::Init(
                    "ADORA_TEST_WITH_INPUTS env variable is not valid unicode".into(),
                ));
            }
            Err(std::env::VarError::NotPresent) => {} // continue trying other init methods
        }

        // interactive mode
        if fallback_to_interactive && std::io::stdin().is_terminal() {
            println!(
                "{}",
                "Starting node in interactive mode as ADORA_NODE_CONFIG env variable is not set"
                    .green()
            );
            return Self::init_interactive();
        }

        // no run mode applicable
        Err(NodeError::Init(
            "ADORA_NODE_CONFIG env variable is not set".into(),
        ))
    }

    /// Initiate a node from a dataflow id and a node id.
    ///
    /// This initialization function should be used for [_dynamic nodes_](index.html#dynamic-nodes).
    ///
    /// ```no_run
    /// use adora_node_api::AdoraNode;
    /// use adora_node_api::adora_core::config::NodeId;
    ///
    /// let (mut node, mut events) = AdoraNode::init_from_node_id(NodeId::from("plot".to_string())).expect("Could not init node plot");
    /// ```
    ///
    pub fn init_from_node_id(node_id: NodeId) -> NodeResult<(Self, EventStream)> {
        // Make sure that the node is initialized outside of adora start.
        let port = match std::env::var(ADORA_DAEMON_LOCAL_LISTEN_PORT_ENV) {
            Ok(p) => p.parse().unwrap_or_else(|e| {
                tracing::warn!(
                    "invalid {ADORA_DAEMON_LOCAL_LISTEN_PORT_ENV}={p:?}: {e}, using default port"
                );
                ADORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT
            }),
            Err(_) => ADORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT,
        };
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
            } => Self::init(node_config),
            DaemonReply::NodeConfig { result: Err(error) } => {
                let capped: String = error.chars().take(512).collect();
                Err(NodeError::Init(format!(
                    "failed to get node config from daemon: {capped}"
                )))
            }
            _ => Err(NodeError::Init("unexpected reply from daemon".into())),
        }
    }

    /// Dynamic initialization function for nodes that are sometimes used as dynamic nodes.
    ///
    /// This function first tries initializing the traditional way through
    /// [`init_from_env`][Self::init_from_env]. If this fails, it falls back to
    /// [`init_from_node_id`][Self::init_from_node_id].
    pub fn init_flexible(node_id: NodeId) -> NodeResult<(Self, EventStream)> {
        if std::env::var("ADORA_NODE_CONFIG").is_ok() {
            info!(
                "Skipping {node_id} specified within the node initialization in favor of `ADORA_NODE_CONFIG` specified by `adora start`"
            );
            Self::init_from_env()
        } else {
            Self::init_from_node_id(node_id)
        }
    }

    /// Initialize the node in a standalone mode that prompts for inputs on the terminal.
    ///
    /// Instead of connecting to a `adora daemon`, this interactive mode prompts for node inputs
    /// on the terminal. In this mode, the node is completely isolated from the adora daemon and
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
    /// Starting node in interactive mode as ADORA_NODE_CONFIG env variable is not set
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
    /// No connection to a adora daemon is made in this mode. Instead, inputs are read from the
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

    /// Internal initialization routine that should not be used outside of Adora.
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
        let shm_pool_size = std::env::var("ADORA_NODE_SHM_POOL_SIZE")
            .ok()
            .and_then(|s| s.parse::<usize>().ok())
            .unwrap_or(8 * 1024 * 1024); // 8 MB default
        let zenoh_zero_copy_threshold = std::env::var("ADORA_ZERO_COPY_THRESHOLD")
            .ok()
            .and_then(|s| s.parse::<usize>().ok())
            .unwrap_or(ZERO_COPY_THRESHOLD);
        let (zenoh_session, zenoh_shm_provider) = if !is_standard_mode {
            (None, None)
        } else {
            match tokio::runtime::Handle::try_current() {
                Ok(handle) => {
                    // Use spawn_blocking + oneshot to avoid panicking when
                    // called from a tokio worker thread (block_on panics in
                    // that context on current-thread runtimes).
                    let session = std::thread::scope(|s| {
                        match s
                            .spawn(|| handle.block_on(adora_core::topics::open_zenoh_session(None)))
                            .join()
                        {
                            Ok(Ok(session)) => Some(session),
                            Ok(Err(e)) => {
                                tracing::warn!("failed to open zenoh session: {e:?}");
                                None
                            }
                            Err(_panic) => {
                                tracing::warn!("zenoh session init panicked");
                                None
                            }
                        }
                    });
                    let provider = if session.is_some() {
                        use zenoh::Wait;
                        use zenoh::shm::ShmProviderBuilder;
                        match ShmProviderBuilder::default_backend(shm_pool_size).wait() {
                            Ok(p) => Some(p),
                            Err(e) => {
                                if std::env::var("ADORA_SHM_REQUIRED").is_ok() {
                                    return Err(NodeError::Init(format!(
                                        "failed to create zenoh SHM provider: {e} \
                                         (ADORA_SHM_REQUIRED is set)"
                                    )));
                                }
                                tracing::warn!(
                                    "failed to create zenoh SHM provider ({e}), \
                                     falling back to heap buffers"
                                );
                                None
                            }
                        }
                    } else {
                        None
                    };
                    (session, provider)
                }
                Err(_) => {
                    tracing::warn!(
                        "no tokio runtime available — zenoh SHM disabled, using daemon path"
                    );
                    (None, None)
                }
            }
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
        )
        .wrap_err("failed to init event stream")?;
        // DropStream tracks custom shmem region lifecycle via DropTokens.
        // When zenoh SHM is available, zenoh handles buffer lifecycle via
        // reference counting — DropTokens are not needed.
        let drop_stream = if zenoh_session.is_some() {
            DropStream::empty()
        } else {
            DropStream::init(dataflow_id, &node_id, &daemon_communication, clock.clone())
                .wrap_err("failed to init drop stream")?
        };
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

        let node = Self {
            id: node_id,
            dataflow_id,
            node_config: run_config.clone(),
            control_channel,
            clock,
            sent_out_shared_memory: HashMap::new(),
            drop_stream,
            cache: VecDeque::new(),
            zenoh_session,
            zenoh_shm_provider,
            zenoh_publishers: HashMap::new(),
            zenoh_zero_copy_threshold,
            dataflow_descriptor: serde_yaml::from_value(dataflow_descriptor),
            warned_unknown_output: BTreeSet::new(),
            interactive: false,
            restart_count,
            runtime_type_checks,
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
        Ok((node, event_stream))
    }

    fn validate_output(&mut self, output_id: &DataId) -> bool {
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
    /// use adora_node_api::{AdoraNode, MetadataParameters};
    /// use adora_core::config::DataId;
    ///
    /// let (mut node, mut events) = AdoraNode::init_from_env().expect("Could not init node.");
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
        let mut sample = self.allocate_data_sample(data_len)?;
        data(&mut sample);

        let type_info = ArrowTypeInfo::byte_array(data_len);

        self.send_output_sample(output_id, type_info, parameters, Some(sample))
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

        // Runtime type check (only when ADORA_RUNTIME_TYPE_CHECK is set).
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

        let framing = self
            .node_config
            .output_framing
            .get(&output_id)
            .copied()
            .unwrap_or_default();

        match framing {
            OutputFraming::Raw => {
                let total_len = required_data_size(&arrow_array);
                let mut sample = self.allocate_data_sample(total_len)?;
                let type_info = copy_array_into_sample(&mut sample, &arrow_array);

                self.send_output_sample(output_id, type_info, parameters, Some(sample))
                    .wrap_err("failed to send output")?;
            }
            OutputFraming::ArrowIpc => {
                let ipc_buf = encode_arrow_ipc(&arrow_array)
                    .map_err(|e| NodeError::Output(format!("Arrow IPC encode: {e}")))?;

                let mut sample = self.allocate_data_sample(ipc_buf.len())?;
                sample.copy_from_slice(&ipc_buf);

                let type_info = ArrowTypeInfo {
                    data_type: arrow_array.data_type().clone(),
                    len: arrow_array.len(),
                    null_count: arrow_array.null_count(),
                    validity: None,
                    offset: 0,
                    buffer_offsets: vec![],
                    child_data: vec![],
                    field_names: None,
                    schema_hash: Some(compute_schema_hash(arrow_array.data_type())),
                };

                let mut parameters = parameters;
                parameters.insert(
                    FRAMING.to_string(),
                    Parameter::String(FRAMING_ARROW_IPC.to_string()),
                );

                self.send_output_sample(output_id, type_info, parameters, Some(sample))
                    .wrap_err("failed to send output")?;
            }
        }

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
        self.send_output_raw(output_id, parameters, data_len, |sample| {
            sample.copy_from_slice(data)
        })
    }

    /// Send the give raw byte data with the provided type information.
    ///
    /// It is recommended to use a function like [`send_output`][Self::send_output] instead.
    ///
    /// Ignores the output if the given `output_id` is not specified as node output in the dataflow
    /// configuration file.
    pub fn send_typed_output<F>(
        &mut self,
        output_id: DataId,
        type_info: ArrowTypeInfo,
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

        let mut sample = self.allocate_data_sample(data_len)?;
        data(&mut sample);

        self.send_output_sample(output_id, type_info, parameters, Some(sample))
    }

    /// Sends the given [`DataSample`] as output, combined with the given type information.
    ///
    /// It is recommended to use a function like [`send_output`][Self::send_output] instead.
    ///
    /// Ignores the output if the given `output_id` is not specified as node output in the dataflow
    /// configuration file.
    pub fn send_output_sample(
        &mut self,
        output_id: DataId,
        type_info: ArrowTypeInfo,
        #[allow(unused_mut)] mut parameters: MetadataParameters,
        sample: Option<DataSample>,
    ) -> NodeResult<()> {
        // Process drop tokens only when using custom shmem (not zenoh SHM).
        // With zenoh, buffer lifecycle is handled by zenoh's reference counting.
        if !self.interactive && self.zenoh_session.is_none() {
            self.handle_finished_drop_tokens()?;
        }

        // Auto-inject OpenTelemetry trace context when telemetry is enabled.
        // Uses the ambient OTel context, which is populated when the tracing
        // subscriber has an OpenTelemetry layer (e.g., via with_otlp_tracing).
        // Only trace/span IDs are propagated (via W3C TraceContext propagator).
        // OTel Baggage is NOT propagated to avoid leaking sensitive data across
        // node boundaries. If a user explicitly provides this key, it wins.
        #[cfg(feature = "tracing")]
        if !parameters.contains_key("open_telemetry_context") {
            let cx = opentelemetry::Context::current();
            let serialized = adora_tracing::telemetry::serialize_context(&cx);
            if !serialized.is_empty() {
                parameters.insert(
                    "open_telemetry_context".to_string(),
                    crate::Parameter::String(serialized),
                );
            }
        }

        let metadata = Metadata::from_parameters(self.clock.new_timestamp(), type_info, parameters);

        let (data, shmem) = match sample {
            Some(sample) => sample.finalize(),
            None => (None, None),
        };

        // Try zenoh SHM publish for data-plane messages.
        // If zenoh session is available, publish data directly via zenoh
        // (zero-copy SHM for local subscribers, network for remote).
        // The daemon still receives a data-less notification for routing awareness.
        let has_zenoh = self.zenoh_session.is_some();
        let zenoh_published = if has_zenoh {
            if let Some(ref raw_data) = data {
                let raw_bytes = match raw_data {
                    DataMessage::Vec(v) => v.as_ref(),
                    DataMessage::SharedMemory { .. } => {
                        // Unreachable when zenoh is active (allocate_data_sample
                        // uses Vec, not custom shmem). Fall back to daemon path
                        // for safety.
                        debug_assert!(
                            !has_zenoh,
                            "DataMessage::SharedMemory should not occur with zenoh active"
                        );
                        &[]
                    }
                };
                if !raw_bytes.is_empty() && raw_bytes.len() >= self.zenoh_zero_copy_threshold {
                    tracing::trace!(
                        output = %output_id,
                        size = raw_bytes.len(),
                        "publishing via zenoh SHM"
                    );
                    match self.zenoh_publish(&output_id, &metadata, raw_bytes) {
                        Ok(()) => true,
                        Err(e) => {
                            tracing::warn!(
                                "zenoh publish failed ({e}), falling back to daemon path"
                            );
                            false
                        }
                    }
                } else {
                    false
                }
            } else {
                false
            }
        } else {
            false
        };

        if zenoh_published {
            // Data delivered directly via zenoh — do NOT send any notification
            // to daemon. The daemon's send_output_to_local_receivers would
            // fan out a data-less NodeEvent::Input to local subscribers,
            // causing duplicate delivery (once from zenoh, once from daemon).
            // The daemon learns about outputs at subscribe time, not per-message.
        } else {
            // Existing path: send data through daemon
            self.control_channel
                .send_message(output_id.clone(), metadata, data)
                .wrap_err_with(|| format!("failed to send output {output_id}"))?;

            if let Some((shared_memory, drop_token)) = shmem {
                self.sent_out_shared_memory
                    .insert(drop_token, shared_memory);
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
        for output_id in &outputs_ids {
            if !self.node_config.outputs.remove(output_id) {
                return Err(NodeError::Output(format!("unknown output {output_id}")));
            }
        }

        self.control_channel
            .report_closed_outputs(outputs_ids)
            .wrap_err("failed to report closed outputs to daemon")?;

        Ok(())
    }

    /// Publish data directly via zenoh (node-to-node, bypassing daemon for data).
    /// Uses SHM for zero-copy when possible, falls back to heap buffer.
    fn zenoh_publish(
        &mut self,
        output_id: &DataId,
        metadata: &Metadata,
        data: &[u8],
    ) -> eyre::Result<()> {
        use zenoh::Wait;

        let session = self.zenoh_session.as_ref().unwrap();

        // Get or create publisher for this output
        if !self.zenoh_publishers.contains_key(output_id) {
            let topic = adora_core::topics::zenoh_output_publish_topic(
                self.dataflow_id,
                &self.id,
                output_id,
            );
            let key_expr = zenoh::key_expr::KeyExpr::new(topic)
                .map_err(|e| eyre::eyre!("invalid zenoh key: {e}"))?
                .into_owned();
            let publisher = session
                .declare_publisher(key_expr)
                .wait()
                .map_err(|e| eyre::eyre!("failed to declare zenoh publisher: {e}"))?;
            self.zenoh_publishers.insert(output_id.clone(), publisher);
        }
        let publisher = self.zenoh_publishers.get(output_id).unwrap();

        // Serialize metadata as zenoh attachment
        let metadata_bytes = bincode::serialize(metadata)
            .wrap_err("failed to serialize metadata for zenoh attachment")?;

        // Try SHM allocation, fall back to heap
        if let Some(provider) = &self.zenoh_shm_provider {
            use zenoh::shm::{BlockOn, GarbageCollect};
            match provider
                .alloc(data.len())
                .with_policy::<BlockOn<GarbageCollect>>()
                .wait()
            {
                Ok(mut sbuf) => {
                    sbuf.as_mut().copy_from_slice(data);
                    publisher
                        .put(sbuf)
                        .attachment(&metadata_bytes[..])
                        .wait()
                        .map_err(|e| eyre::eyre!("zenoh SHM publish failed: {e}"))?;
                    return Ok(());
                }
                Err(e) => {
                    tracing::debug!("SHM alloc failed ({e}), using heap buffer");
                }
            }
        }

        // Fallback: publish raw bytes (no SHM)
        publisher
            .put(data)
            .attachment(&metadata_bytes[..])
            .wait()
            .map_err(|e| eyre::eyre!("zenoh publish failed: {e}"))?;

        Ok(())
    }

    /// Returns the ID of the node as specified in the dataflow configuration file.
    pub fn id(&self) -> &NodeId {
        &self.id
    }

    /// Returns the unique identifier for the running dataflow instance.
    ///
    /// Adora assigns each dataflow instance a random identifier when started.
    pub fn dataflow_id(&self) -> &DataflowId {
        &self.dataflow_id
    }

    /// Returns the input and output configuration of this node.
    pub fn node_config(&self) -> &NodeRunConfig {
        &self.node_config
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
                eprintln!("adora log: fields too large ({total} bytes), dropping fields");
                entry["fields_dropped"] = serde_json::Value::Bool(true);
            }
        }
        match serde_json::to_string(&entry) {
            Ok(json) => println!("{json}"),
            Err(e) => eprintln!("adora log serialization error: {e}"),
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
        if parameters.contains_key(adora_message::metadata::REQUEST_ID) {
            tracing::warn!("send_service_request: caller-provided request_id will be overwritten");
        }
        let request_id = Self::new_request_id();
        parameters.insert(
            adora_message::metadata::REQUEST_ID.to_string(),
            adora_message::metadata::Parameter::String(request_id.clone()),
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
    /// The data sample will use shared memory when suitable to enable efficient data transfer
    /// when sending an output message.
    pub fn allocate_data_sample(&mut self, data_len: usize) -> NodeResult<DataSample> {
        // When zenoh SHM is active, always use Vec allocation (not custom shmem).
        // Zenoh handles zero-copy via its own SHM pool in zenoh_publish().
        // Using custom shmem would create DataMessage::SharedMemory which can't
        // be published via zenoh, and whose DropTokens would never be drained
        // (DropStream::empty() when zenoh is active).
        let use_custom_shmem = self.zenoh_session.is_none();
        let data = if data_len >= ZERO_COPY_THRESHOLD && !self.interactive && use_custom_shmem {
            // create shared memory region
            let shared_memory = self.allocate_shared_memory(data_len)?;

            DataSample {
                inner: DataSampleInner::Shmem(shared_memory),
                len: data_len,
            }
        } else {
            let avec: AVec<u8, ConstAlign<128>> = AVec::__from_elem(128, 0, data_len);

            avec.into()
        };

        Ok(data)
    }

    fn allocate_shared_memory(&mut self, data_len: usize) -> eyre::Result<ShmemHandle> {
        let cache_index = self
            .cache
            .iter()
            .enumerate()
            .rev()
            .filter(|(_, s)| s.len() >= data_len)
            .min_by_key(|(_, s)| s.len())
            .map(|(i, _)| i);
        let memory = match cache_index {
            Some(i) => {
                // we know that this index exists, so we can safely unwrap here
                self.cache.remove(i).unwrap()
            }
            None => ShmemHandle(Box::new(
                ShmemConf::new()
                    .size(data_len)
                    .writable(true)
                    .create()
                    .wrap_err("failed to allocate shared memory")?,
            )),
        };
        assert!(memory.len() >= data_len);

        Ok(memory)
    }

    fn handle_finished_drop_tokens(&mut self) -> eyre::Result<()> {
        loop {
            match self.drop_stream.try_recv() {
                Ok(token) => match self.sent_out_shared_memory.remove(&token) {
                    Some(region) => self.add_to_cache(region),
                    None => tracing::warn!("received unknown finished drop token `{token:?}`"),
                },
                Err(flume::TryRecvError::Empty) => break,
                Err(flume::TryRecvError::Disconnected) => {
                    bail!("event stream was closed before sending all expected drop tokens")
                }
            }
        }
        Ok(())
    }

    fn add_to_cache(&mut self, memory: ShmemHandle) {
        const MAX_CACHE_COUNT: usize = 20;
        /// Maximum total bytes held in the shared memory cache (256 MB).
        const MAX_CACHE_BYTES: usize = 256 * 1024 * 1024;

        self.cache.push_back(memory);

        // Evict oldest entries if over count limit
        while self.cache.len() > MAX_CACHE_COUNT {
            self.cache.pop_front();
        }

        // Evict oldest entries if over byte budget
        let total_bytes: usize = self.cache.iter().map(|h| h.len()).sum();
        if total_bytes > MAX_CACHE_BYTES {
            self.cache.clear();
        }
    }

    /// Returns the full dataflow descriptor that this node is part of.
    ///
    /// This method returns the parsed dataflow YAML file.
    pub fn dataflow_descriptor(&self) -> NodeResult<&Descriptor> {
        match &self.dataflow_descriptor {
            Ok(d) => Ok(d),
            Err(err) => Err(NodeError::Data(format!(
                "failed to parse dataflow descriptor: {err}\n\n\
                    This might be caused by mismatched version numbers of adora \
                    daemon and the adora node API"
            ))),
        }
    }
}

impl Drop for AdoraNode {
    fn drop(&mut self) {
        // Undeclare zenoh publishers to clean up network resources
        // before closing daemon channels.
        self.zenoh_publishers.clear();
        // Drop the session explicitly (releases SHM pool + network resources)
        self.zenoh_shm_provider.take();
        self.zenoh_session.take();

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

        while !self.sent_out_shared_memory.is_empty() {
            if self.drop_stream.is_empty() {
                tracing::trace!(
                    "waiting for {} remaining drop tokens",
                    self.sent_out_shared_memory.len()
                );
            }

            match self.drop_stream.recv_timeout(Duration::from_secs(2)) {
                Ok(token) => {
                    self.sent_out_shared_memory.remove(&token);
                }
                Err(flume::RecvTimeoutError::Disconnected) => {
                    tracing::warn!(
                        "finished_drop_tokens channel closed while still waiting for drop tokens; \
                        closing {} shared memory regions that might not yet been mapped.",
                        self.sent_out_shared_memory.len()
                    );
                    break;
                }
                Err(flume::RecvTimeoutError::Timeout) => {
                    tracing::warn!(
                        "timeout while waiting for drop tokens; \
                        closing {} shared memory regions that might not yet been mapped.",
                        self.sent_out_shared_memory.len()
                    );
                    break;
                }
            }
        }

        if let Err(err) = self.control_channel.report_outputs_done() {
            tracing::warn!("{err:?}")
        }
    }
}

/// A data region suitable for sending as an output message.
///
/// The region is stored in shared memory when suitable to enable efficient data transfer.
///
/// `DataSample` implements the [`Deref`] and [`DerefMut`] traits to read and write the mapped data.
pub struct DataSample {
    inner: DataSampleInner,
    len: usize,
}

impl DataSample {
    fn finalize(self) -> (Option<DataMessage>, Option<(ShmemHandle, DropToken)>) {
        match self.inner {
            DataSampleInner::Shmem(shared_memory) => {
                let drop_token = DropToken::generate();
                let data = DataMessage::SharedMemory {
                    shared_memory_id: shared_memory.get_os_id().to_owned(),
                    len: self.len,
                    drop_token,
                };
                (Some(data), Some((shared_memory, drop_token)))
            }
            DataSampleInner::Vec(buffer) => (Some(DataMessage::Vec(buffer)), None),
        }
    }
}

impl Deref for DataSample {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        let slice = match &self.inner {
            DataSampleInner::Shmem(handle) => unsafe { handle.as_slice() },
            DataSampleInner::Vec(data) => data,
        };
        &slice[..self.len]
    }
}

impl DerefMut for DataSample {
    fn deref_mut(&mut self) -> &mut Self::Target {
        let slice = match &mut self.inner {
            DataSampleInner::Shmem(handle) => unsafe { handle.as_slice_mut() },
            DataSampleInner::Vec(data) => data,
        };
        &mut slice[..self.len]
    }
}

impl From<AVec<u8, ConstAlign<128>>> for DataSample {
    fn from(value: AVec<u8, ConstAlign<128>>) -> Self {
        Self {
            len: value.len(),
            inner: DataSampleInner::Vec(value),
        }
    }
}

impl std::fmt::Debug for DataSample {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let kind = match &self.inner {
            DataSampleInner::Shmem(_) => "SharedMemory",
            DataSampleInner::Vec(_) => "Vec",
        };
        f.debug_struct("DataSample")
            .field("len", &self.len)
            .field("kind", &kind)
            .finish_non_exhaustive()
    }
}

enum DataSampleInner {
    Shmem(ShmemHandle),
    Vec(AVec<u8, ConstAlign<128>>),
}

struct ShmemHandle(Box<Shmem>);

impl Deref for ShmemHandle {
    type Target = Shmem;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for ShmemHandle {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

unsafe impl Send for ShmemHandle {}
unsafe impl Sync for ShmemHandle {}

/// Returns `true` if the given metadata carries any pattern-correlation
/// key (`request_id`, `goal_id`, or `goal_status`).
///
/// Messages marked with these keys belong to a service, action, or
/// streaming pattern where multiple Arrow schemas can legitimately
/// flow through a single output/input, distinguished by metadata
/// rather than a fixed Arrow type. Type checks are skipped for such
/// messages (dora-rs/adora#150).
pub(crate) fn carries_pattern_correlation(params: &MetadataParameters) -> bool {
    params.contains_key(adora_message::metadata::REQUEST_ID)
        || params.contains_key(adora_message::metadata::GOAL_ID)
        || params.contains_key(adora_message::metadata::GOAL_STATUS)
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
        if std::env::var("ADORA_OTLP_ENDPOINT").is_ok()
            || std::env::var("ADORA_JAEGER_TRACING").is_ok()
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
    if std::env::var("ADORA_OTLP_ENDPOINT").is_ok() {
        let id = format!("{dataflow_id}/{node_id}");
        let monitor_task = async move {
            use adora_metrics::run_metrics_monitor;

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
            session_id: AdoraNode::new_request_id(),
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
        let id = AdoraNode::new_request_id();
        uuid::Uuid::parse_str(&id).expect("should be valid UUID");
    }

    #[test]
    fn new_request_id_is_unique() {
        let ids: Vec<String> = (0..100).map(|_| AdoraNode::new_request_id()).collect();
        let unique: std::collections::HashSet<_> = ids.iter().collect();
        assert_eq!(ids.len(), unique.len(), "all IDs should be unique");
    }

    #[test]
    fn new_goal_id_returns_valid_uuid() {
        let id = AdoraNode::new_goal_id();
        uuid::Uuid::parse_str(&id).expect("should be valid UUID");
    }

    /// Helper: create a minimal test node with a channel output.
    fn test_node() -> (
        AdoraNode,
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
        let (node, event_stream) = AdoraNode::init_testing(inputs, outputs, options).unwrap();
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
            adora_message::metadata::REQUEST_ID.to_string(),
            adora_message::metadata::Parameter::String("test-req-id".into()),
        );
        node.send_service_response("response".into(), params, NullArray::new(0))
            .unwrap();

        drop(node);
        drop(events);
        let outputs: Vec<_> = rx.try_iter().collect();
        assert_eq!(outputs.len(), 1);
        assert_eq!(outputs[0]["id"], "response");
    }

    // ---- dora-rs/adora#150: pattern polymorphism exemption ----

    #[test]
    fn carries_pattern_correlation_detects_request_id() {
        let mut params = MetadataParameters::default();
        params.insert(
            adora_message::metadata::REQUEST_ID.to_string(),
            adora_message::metadata::Parameter::String("req-1".into()),
        );
        assert!(carries_pattern_correlation(&params));
    }

    #[test]
    fn carries_pattern_correlation_detects_goal_id() {
        let mut params = MetadataParameters::default();
        params.insert(
            adora_message::metadata::GOAL_ID.to_string(),
            adora_message::metadata::Parameter::String("goal-1".into()),
        );
        assert!(carries_pattern_correlation(&params));
    }

    #[test]
    fn carries_pattern_correlation_detects_goal_status() {
        let mut params = MetadataParameters::default();
        params.insert(
            adora_message::metadata::GOAL_STATUS.to_string(),
            adora_message::metadata::Parameter::String("succeeded".into()),
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
            adora_message::metadata::Parameter::String("value".into()),
        );
        assert!(!carries_pattern_correlation(&params));
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
}
