use crate::{
    DaemonCommunicationWrapper, EventStream,
    daemon_connection::{DaemonChannel, IntegrationTestingEvents},
    integration_testing::{
        TestingCommunication, TestingInput, TestingOptions, TestingOutput,
        take_testing_communication,
    },
};

use self::{
    arrow_utils::{copy_array_into_sample, required_data_size},
    control_channel::ControlChannel,
    drop_stream::DropStream,
};
use aligned_vec::{AVec, ConstAlign};
use arrow::array::Array;
use colored::Colorize;
use dora_core::{
    config::{DataId, NodeId, NodeRunConfig},
    descriptor::Descriptor,
    metadata::ArrowTypeInfoExt,
    topics::{DORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT, LOCALHOST},
    uhlc,
};
use dora_message::{
    DataflowId,
    daemon_to_node::{DaemonCommunication, DaemonReply, NodeConfig},
    metadata::{ArrowTypeInfo, Metadata, MetadataParameters},
    node_to_daemon::{DaemonRequest, DataMessage, DropToken, Timestamped},
};
use eyre::{WrapErr, bail, eyre};
use is_terminal::IsTerminal;
use shared_memory_extended::{Shmem, ShmemConf};
use std::{
    collections::{BTreeSet, HashMap, VecDeque},
    ops::{Deref, DerefMut},
    path::PathBuf,
    sync::Arc,
    time::{Duration, Instant},
};
use tokio::runtime::{Handle, Runtime};
use tracing::{info, warn};

#[cfg(feature = "metrics")]
use dora_metrics::run_metrics_monitor;
#[cfg(feature = "tracing")]
use dora_tracing::TracingBuilder;

pub mod arrow_utils;
mod control_channel;
mod drop_stream;

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

#[allow(dead_code)]
enum TokioRuntime {
    Runtime(Runtime),
    Handle(Handle),
}

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

    sent_out_shared_memory: HashMap<DropToken, ShmemHandle>,
    drop_stream: DropStream,
    cache: VecDeque<ShmemHandle>,

    dataflow_descriptor: serde_yaml::Result<Descriptor>,
    warned_unknown_output: BTreeSet<DataId>,
    _rt: TokioRuntime,

    interactive: bool,

    // Health observability tracking
    input_last_received: HashMap<DataId, std::time::Instant>,
    input_closed: BTreeSet<DataId>,
    input_timeout_callbacks: HashMap<DataId, (Duration, Arc<dyn Fn() + Send + Sync>)>,
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
    pub fn init_from_env() -> eyre::Result<(Self, EventStream)> {
        Self::init_from_env_inner(true)
    }

    /// Initialize the node from environment variables set by the Dora daemon; error if not set.
    ///
    /// This function behaves the same as [`init_from_env`](Self::init_from_env), but it does _not_
    /// fall back to [`init_interactive`](Self::init_interactive). Instead, an error is returned
    /// when the `DORA_NODE_CONFIG` environment variable is missing.
    pub fn init_from_env_force() -> eyre::Result<(Self, EventStream)> {
        Self::init_from_env_inner(false)
    }

    fn init_from_env_inner(fallback_to_interactive: bool) -> eyre::Result<(Self, EventStream)> {
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
                #[cfg(feature = "tracing")]
                {
                    TracingBuilder::new(node_config.node_id.as_ref())
                        .with_stdout("info", true)
                        .build()
                        .wrap_err("failed to set up tracing subscriber")?;
                }

                return Self::init(node_config);
            }
            Err(std::env::VarError::NotUnicode(_)) => {
                bail!("DORA_NODE_CONFIG env variable is not valid unicode")
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
                        bail!("DORA_TEST_WRITE_OUTPUTS_TO env variable is not valid unicode")
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
                bail!("DORA_TEST_WITH_INPUTS env variable is not valid unicode")
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
        bail!("DORA_NODE_CONFIG env variable is not set")
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
    pub fn init_from_node_id(node_id: NodeId) -> eyre::Result<(Self, EventStream)> {
        // Make sure that the node is initialized outside of dora start.
        let daemon_address = (LOCALHOST, DORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT).into();

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
                bail!("failed to get node config from daemon: {error}")
            }
            _ => bail!("unexpected reply from daemon"),
        }
    }

    /// Dynamic initialization function for nodes that are sometimes used as dynamic nodes.
    ///
    /// This function first tries initializing the traditional way through
    /// [`init_from_env`][Self::init_from_env]. If this fails, it falls back to
    /// [`init_from_node_id`][Self::init_from_node_id].
    pub fn init_flexible(node_id: NodeId) -> eyre::Result<(Self, EventStream)> {
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
    pub fn init_interactive() -> eyre::Result<(Self, EventStream)> {
        #[cfg(feature = "tracing")]
        {
            TracingBuilder::new("node")
                .with_stdout("debug", false)
                .build()
                .wrap_err("failed to set up tracing subscriber")?;
        }

        let node_config = NodeConfig {
            dataflow_id: DataflowId::new_v4(),
            node_id: "".parse()?,
            run_config: NodeRunConfig {
                inputs: Default::default(),
                outputs: Default::default(),
            },
            daemon_communication: Some(DaemonCommunication::Interactive),
            dataflow_descriptor: serde_yaml::Value::Null,
            dynamic: false,
            write_events_to: None,
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
    ) -> eyre::Result<(Self, EventStream)> {
        let node_config = NodeConfig {
            dataflow_id: DataflowId::new_v4(),
            node_id: "".parse()?,
            run_config: NodeRunConfig {
                inputs: Default::default(),
                outputs: Default::default(),
            },
            daemon_communication: None,
            dataflow_descriptor: serde_yaml::Value::Null,
            dynamic: false,
            write_events_to: None,
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
    pub fn init(node_config: NodeConfig) -> eyre::Result<(Self, EventStream)> {
        Self::init_with_options(node_config, None)
    }

    #[tracing::instrument(skip(testing_communication))]
    fn init_with_options(
        node_config: NodeConfig,
        testing_communication: Option<TestingCommunication>,
    ) -> eyre::Result<(Self, EventStream)> {
        let NodeConfig {
            dataflow_id,
            node_id,
            run_config,
            daemon_communication,
            dataflow_descriptor,
            dynamic,
            write_events_to,
        } = node_config;
        let clock = Arc::new(uhlc::HLC::default());
        let input_config = run_config.inputs.clone();

        let rt = match Handle::try_current() {
            Ok(handle) => TokioRuntime::Handle(handle),
            Err(_) => TokioRuntime::Runtime(
                tokio::runtime::Builder::new_multi_thread()
                    .worker_threads(2)
                    .enable_all()
                    .build()
                    .context("tokio runtime failed")?,
            ),
        };

        #[cfg(feature = "metrics")]
        {
            let id = format!("{dataflow_id}/{node_id}");
            let monitor_task = async move {
                if let Err(e) = run_metrics_monitor(id.clone())
                    .await
                    .wrap_err("metrics monitor exited unexpectedly")
                {
                    warn!("metrics monitor failed: {:#?}", e);
                }
            };
            match &rt {
                TokioRuntime::Runtime(rt) => rt.spawn(monitor_task),
                TokioRuntime::Handle(handle) => handle.spawn(monitor_task),
            };
        }

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
                None => eyre::bail!("no daemon communication method specified"),
            },
        };

        let event_stream = EventStream::init(
            dataflow_id,
            &node_id,
            &daemon_communication,
            input_config,
            clock.clone(),
            write_events_to,
        )
        .wrap_err("failed to init event stream")?;
        let drop_stream =
            DropStream::init(dataflow_id, &node_id, &daemon_communication, clock.clone())
                .wrap_err("failed to init drop stream")?;
        let control_channel =
            ControlChannel::init(dataflow_id, &node_id, &daemon_communication, clock.clone())
                .wrap_err("failed to init control channel")?;

        let node = Self {
            id: node_id,
            dataflow_id,
            node_config: run_config.clone(),
            control_channel,
            clock,
            sent_out_shared_memory: HashMap::new(),
            drop_stream,
            cache: VecDeque::new(),
            dataflow_descriptor: serde_yaml::from_value(dataflow_descriptor),
            warned_unknown_output: BTreeSet::new(),
            _rt: rt,
            interactive: false,
            input_last_received: HashMap::new(),
            input_closed: BTreeSet::new(),
            input_timeout_callbacks: HashMap::new(),
        };

        if dynamic {
            // Inject env variable from dataflow descriptor.
            match &node.dataflow_descriptor {
                Ok(descriptor) => {
                    if let Some(env_vars) = descriptor
                        .nodes
                        .iter()
                        .find(|n| n.id == node.id)
                        .and_then(|n| n.env.as_ref())
                    {
                        for (key, value) in env_vars {
                            // SAFETY: setting env variable is safe as long as we don't
                            // have multiple threads doing it at the same time.
                            unsafe {
                                std::env::set_var(key, value.to_string());
                            }
                        }
                    }
                }
                Err(err) => {
                    warn!("Could not parse dataflow descriptor: {err:#}");
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
    ) -> eyre::Result<()>
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
    ) -> eyre::Result<()> {
        if !self.validate_output(&output_id) {
            return Ok(());
        };

        let arrow_array = data.to_data();

        let total_len = required_data_size(&arrow_array);

        let mut sample = self.allocate_data_sample(total_len)?;
        let type_info = copy_array_into_sample(&mut sample, &arrow_array);

        self.send_output_sample(output_id, type_info, parameters, Some(sample))
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
    ) -> eyre::Result<()> {
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
    ) -> eyre::Result<()>
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
        parameters: MetadataParameters,
        sample: Option<DataSample>,
    ) -> eyre::Result<()> {
        if !self.interactive {
            self.handle_finished_drop_tokens()?;
        }

        let metadata = Metadata::from_parameters(self.clock.new_timestamp(), type_info, parameters);

        let (data, shmem) = match sample {
            Some(sample) => sample.finalize(),
            None => (None, None),
        };

        self.control_channel
            .send_message(output_id.clone(), metadata, data)
            .wrap_err_with(|| format!("failed to send output {output_id}"))?;

        if let Some((shared_memory, drop_token)) = shmem {
            self.sent_out_shared_memory
                .insert(drop_token, shared_memory);
        }

        Ok(())
    }

    /// Report the given outputs IDs as closed.
    ///
    /// The node is not allowed to send more outputs with the closed IDs.
    ///
    /// Closing outputs early can be helpful to receivers.
    pub fn close_outputs(&mut self, outputs_ids: Vec<DataId>) -> eyre::Result<()> {
        for output_id in &outputs_ids {
            if !self.node_config.outputs.remove(output_id) {
                eyre::bail!("unknown output {output_id}");
            }
        }

        self.control_channel
            .report_closed_outputs(outputs_ids)
            .wrap_err("failed to report closed outputs to daemon")?;

        Ok(())
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

    /// Subscribe to lifecycle events of other nodes.
    ///
    /// After subscribing, the node will receive [`Event::PeerStarted`], [`Event::PeerStopped`],
    /// and [`Event::PeerHealthChanged`] events for the specified nodes.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use dora_node_api::DoraNode;
    /// use dora_core::config::NodeId;
    ///
    /// let (mut node, mut events) = DoraNode::init_from_env()?;
    /// node.subscribe_node_events(&[
    ///     NodeId::from("camera_node".to_string()),
    ///     NodeId::from("detector_node".to_string()),
    /// ])?;
    /// # Ok::<(), eyre::Report>(())
    /// ```
    pub fn subscribe_node_events(&mut self, node_ids: &[NodeId]) -> eyre::Result<()> {
        self.control_channel
            .request(&Timestamped {
                inner: DaemonRequest::SubscribeNodeEvents {
                    node_ids: node_ids.to_vec(),
                },
                timestamp: self.clock.new_timestamp(),
            })
            .wrap_err("failed to subscribe to node events")?;
        Ok(())
    }

    /// Set this node's health status.
    ///
    /// This allows nodes to actively declare their health status to the framework.
    /// Other nodes that have subscribed to this node's lifecycle events will be notified.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use dora_node_api::DoraNode;
    /// use dora_message::common::HealthStatus;
    ///
    /// let (mut node, mut events) = DoraNode::init_from_env()?;
    /// // Declare degraded status due to high latency
    /// node.set_health_status(HealthStatus::Degraded)?;
    /// # Ok::<(), eyre::Report>(())
    /// ```
    pub fn set_health_status(
        &mut self,
        status: dora_message::common::HealthStatus,
    ) -> eyre::Result<()> {
        self.control_channel
            .send(&Timestamped {
                inner: DaemonRequest::SetHealthStatus { status },
                timestamp: self.clock.new_timestamp(),
            })
            .wrap_err("failed to set health status")?;
        Ok(())
    }

    /// Send an error event to downstream nodes instead of crashing.
    ///
    /// This allows nodes to propagate errors gracefully without terminating,
    /// giving downstream nodes a chance to handle the error.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use dora_node_api::DoraNode;
    /// use dora_core::config::DataId;
    ///
    /// let (mut node, mut events) = DoraNode::init_from_env()?;
    /// let output_id = DataId::from("result".to_string());
    /// node.send_error(output_id, "Processing failed: invalid input")?;
    /// # Ok::<(), eyre::Report>(())
    /// ```
    pub fn send_error(&mut self, output_id: DataId, error: impl Into<String>) -> eyre::Result<()> {
        if !self.validate_output(&output_id) {
            return Ok(());
        }

        self.control_channel
            .send(&Timestamped {
                inner: DaemonRequest::SendError {
                    output_id,
                    error: error.into(),
                },
                timestamp: self.clock.new_timestamp(),
            })
            .wrap_err("failed to send error event")?;
        Ok(())
    }

    /// Query the health status of an input.
    ///
    /// This can be used to check if an upstream node is still healthy and sending data.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use dora_node_api::DoraNode;
    /// use dora_core::config::DataId;
    /// use dora_message::daemon_to_node::InputHealth;
    ///
    /// let (mut node, mut events) = DoraNode::init_from_env()?;
    /// let input_id = DataId::from("image".to_string());
    /// let health = node.query_input_health(&input_id)?;
    /// if health == InputHealth::Timeout {
    ///     println!("Input has timed out, using cached data");
    /// }
    /// # Ok::<(), eyre::Report>(())
    /// ```
    pub fn query_input_health(
        &mut self,
        input_id: &DataId,
    ) -> eyre::Result<dora_message::daemon_to_node::InputHealth> {
        let reply = self
            .control_channel
            .request(&Timestamped {
                inner: DaemonRequest::QueryInputHealth {
                    input_id: input_id.clone(),
                },
                timestamp: self.clock.new_timestamp(),
            })
            .wrap_err("failed to query input health")?;

        match reply {
            DaemonReply::InputHealth { result } => {
                result.map_err(|e| eyre!("failed to get input health status: {}", e))
            }
            _ => bail!("unexpected reply from daemon: expected InputHealth"),
        }
    }

    /// Check if an input is still alive (receiving data).
    ///
    /// Returns `true` if the input has received data and hasn't been closed.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use dora_node_api::DoraNode;
    /// use dora_core::config::DataId;
    ///
    /// let (mut node, mut events) = DoraNode::init_from_env()?;
    /// let input_id = DataId::from("image".to_string());
    /// if !node.input_is_alive(&input_id) {
    ///     println!("Input is no longer active");
    /// }
    /// # Ok::<(), eyre::Report>(())
    /// ```
    pub fn input_is_alive(&self, input_id: &DataId) -> bool {
        !self.input_closed.contains(input_id) && self.input_last_received.contains_key(input_id)
    }

    /// Get the last time data was received on an input.
    ///
    /// Returns `None` if no data has been received yet.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use dora_node_api::DoraNode;
    /// use dora_core::config::DataId;
    /// use std::time::Duration;
    ///
    /// let (mut node, mut events) = DoraNode::init_from_env()?;
    /// let input_id = DataId::from("image".to_string());
    /// if let Some(last_time) = node.input_last_received_time(&input_id) {
    ///     if last_time.elapsed() > Duration::from_secs(5) {
    ///         println!("Input timeout detected!");
    ///     }
    /// }
    /// # Ok::<(), eyre::Report>(())
    /// ```
    pub fn input_last_received_time(&self, input_id: &DataId) -> Option<std::time::Instant> {
        self.input_last_received.get(input_id).copied()
    }

    /// Set a timeout callback for an input.
    ///
    /// The callback will be invoked when the input hasn't received data for the specified duration.
    /// The timeout is checked when processing events, so the callback may not fire immediately.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use dora_node_api::DoraNode;
    /// use dora_core::config::DataId;
    /// use std::time::Duration;
    ///
    /// let (mut node, mut events) = DoraNode::init_from_env()?;
    /// let input_id = DataId::from("camera".to_string());
    ///
    /// node.set_input_timeout(input_id.clone(), Duration::from_secs(5), || {
    ///     eprintln!("Camera feed timeout! Switching to backup...");
    /// });
    /// # Ok::<(), eyre::Report>(())
    /// ```
    pub fn set_input_timeout<F>(&mut self, input_id: DataId, duration: Duration, callback: F)
    where
        F: Fn() + Send + Sync + 'static,
    {
        self.input_timeout_callbacks
            .insert(input_id, (duration, Arc::new(callback)));
    }

    /// Check and invoke timeout callbacks for inputs that haven't received data.
    ///
    /// This is called internally by the event stream.
    #[doc(hidden)]
    pub fn _check_timeout_callbacks(&self) {
        let now = std::time::Instant::now();
        for (input_id, (timeout_duration, callback)) in &self.input_timeout_callbacks {
            if let Some(last_received) = self.input_last_received.get(input_id) {
                if now.duration_since(*last_received) > *timeout_duration {
                    callback();
                }
            }
        }
    }

    /// Update input tracking when data is received (internal use).
    ///
    /// This should be called by the event stream when input data arrives.
    #[doc(hidden)]
    pub fn _update_input_received(&mut self, input_id: &DataId) {
        self.input_last_received
            .insert(input_id.clone(), std::time::Instant::now());
    }

    /// Mark an input as closed (internal use).
    ///
    /// This should be called by the event stream when an input is closed.
    #[doc(hidden)]
    pub fn _mark_input_closed(&mut self, input_id: &DataId) {
        self.input_closed.insert(input_id.clone());
    }

    /// Allocates a [`DataSample`] of the specified size.
    ///
    /// The data sample will use shared memory when suitable to enable efficient data transfer
    /// when sending an output message.
    pub fn allocate_data_sample(&mut self, data_len: usize) -> eyre::Result<DataSample> {
        let data = if data_len >= ZERO_COPY_THRESHOLD && !self.interactive {
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
        const MAX_CACHE_SIZE: usize = 20;

        self.cache.push_back(memory);
        while self.cache.len() > MAX_CACHE_SIZE {
            self.cache.pop_front();
        }
    }

    /// Returns the full dataflow descriptor that this node is part of.
    ///
    /// This method returns the parsed dataflow YAML file.
    pub fn dataflow_descriptor(&self) -> eyre::Result<&Descriptor> {
        match &self.dataflow_descriptor {
            Ok(d) => Ok(d),
            Err(err) => eyre::bail!(
                "failed to parse dataflow descriptor: {err}\n\n
                This might be caused by mismatched version numbers of dora \
                daemon and the dora node API"
            ),
        }
    }
}

impl Drop for DoraNode {
    #[tracing::instrument(skip(self), fields(self.id = %self.id), level = "trace")]
    fn drop(&mut self) {
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
