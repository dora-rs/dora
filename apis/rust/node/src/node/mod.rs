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
    node_to_daemon::{DaemonRequest, DataMessage, Timestamped},
};
use eyre::{WrapErr, bail};
use is_terminal::IsTerminal;

use std::sync::Mutex;
use std::{
    collections::{BTreeSet, HashMap},
    ops::{Deref, DerefMut},
    path::PathBuf,
    sync::Arc,
};
use tokio::runtime::Handle;

#[cfg(feature = "tracing")]
use dora_tracing::{OtelGuard, TracingBuilder};
use tracing::{info, warn};

pub mod arrow_utils;
mod control_channel;

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

    dataflow_descriptor: serde_yaml::Result<Descriptor>,
    warned_unknown_output: BTreeSet<DataId>,
    interactive: bool,

    zenoh_session: Option<zenoh::Session>,
    shm_provider: Option<zenoh::shm::ShmProvider<zenoh::shm::PosixShmProviderBackend>>,
    publishers: HashMap<DataId, zenoh::pubsub::Publisher<'static>>,
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
                shared_memory_pool_size: None,
            },
            daemon_communication: Some(DaemonCommunication::Interactive),
            dataflow_descriptor: serde_yaml::Value::Null,
            dynamic: false,
            write_events_to: None,
            coordinator_addr: None,
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
                shared_memory_pool_size: None,
            },
            daemon_communication: None,
            dataflow_descriptor: serde_yaml::Value::Null,
            dynamic: false,
            write_events_to: None,
            coordinator_addr: None,
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
            coordinator_addr,
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
                None => eyre::bail!("no daemon communication method specified"),
            },
        };

        // Set up zenoh session for data plane (unless interactive/testing)
        let is_interactive = matches!(
            &daemon_communication,
            DaemonCommunicationWrapper::Testing { .. }
        ) || matches!(
            &daemon_communication,
            DaemonCommunicationWrapper::Standard(DaemonCommunication::Interactive)
        );

        let zenoh_session = if !is_interactive {
            let session = dora_core::topics::open_zenoh_session_sync(coordinator_addr)
                .wrap_err("failed to open zenoh session for node")?;
            Some(session)
        } else {
            None
        };

        let event_stream = EventStream::init(
            dataflow_id,
            &node_id,
            &daemon_communication,
            input_config,
            clock.clone(),
            write_events_to,
            zenoh_session.as_ref(),
        )
        .wrap_err("failed to init event stream")?;
        let control_channel =
            ControlChannel::init(dataflow_id, &node_id, &daemon_communication, clock.clone())
                .wrap_err("failed to init control channel")?;

        let has_outputs = !run_config.outputs.is_empty();
        let (shm_provider, publishers) = if zenoh_session.is_some() && has_outputs {
            use zenoh::Wait;
            let session = zenoh_session.as_ref().unwrap();

            // Create SHM provider with configurable pool size.
            // The pool is used for zero-copy output publishing. If the pool is
            // exhausted, allocation blocks until receivers release buffers (see
            // `allocate_data_sample`). For high-throughput nodes with large
            // messages, configure via `shared_memory_pool_size` in the dataflow
            // YAML or the `DORA_NODE_SHM_POOL_SIZE` env var (bytes).
            let pool_size: usize = run_config
                .shared_memory_pool_size
                .map(|bs| bs.as_bytes())
                .or_else(|| {
                    std::env::var("DORA_NODE_SHM_POOL_SIZE")
                        .ok()
                        .and_then(|s| s.parse().ok())
                })
                .unwrap_or(8 * 1024 * 1024); // 8MB default

            let provider = {
                use zenoh::shm::*;
                match ShmProviderBuilder::default_backend(pool_size).wait() {
                    Ok(p) => Some(p),
                    Err(e) => {
                        if std::env::var("DORA_SHM_REQUIRED").is_ok() {
                            return Err(eyre::eyre!("{e}")).wrap_err(
                                "failed to create zenoh SHM provider (DORA_SHM_REQUIRED is set)",
                            );
                        }
                        tracing::warn!(
                            "failed to create zenoh SHM provider ({e}), falling back to heap buffers"
                        );
                        None
                    }
                }
            };

            // Declare publishers for each output
            let mut pubs = HashMap::new();
            for output_id in run_config.outputs.iter() {
                let topic =
                    dora_core::topics::zenoh_output_publish_topic(dataflow_id, &node_id, output_id);
                let publisher = session
                    .declare_publisher(topic)
                    .wait()
                    .map_err(|e| eyre::eyre!("{e}"))
                    .wrap_err_with(|| {
                        format!("failed to declare zenoh publisher for output {output_id}")
                    })?;
                pubs.insert(output_id.clone(), publisher);
            }

            (provider, pubs)
        } else {
            (None, HashMap::new())
        };

        let node = Self {
            id: node_id,
            dataflow_id,
            node_config: run_config.clone(),
            control_channel,
            clock,
            dataflow_descriptor: serde_yaml::from_value(dataflow_descriptor),
            warned_unknown_output: BTreeSet::new(),
            interactive: false,
            zenoh_session,
            shm_provider,
            publishers,
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
        let metadata = Metadata::from_parameters(self.clock.new_timestamp(), type_info, parameters);

        // If we have a zenoh publisher for this output, publish via zenoh and
        // notify the daemon with a data-less control message. Otherwise fall back
        // to sending the full payload through the control channel.
        let data = if let Some(publisher) = self.publishers.get(&output_id) {
            if let Some(sample) = sample {
                use zenoh::Wait;

                // Serialize metadata as a zenoh attachment so receivers get both
                // the data payload (zero-copy via SHM) and the metadata.
                let metadata_bytes = bincode::serialize(&metadata)
                    .wrap_err("failed to serialize metadata for zenoh attachment")?;

                match sample.inner {
                    DataSampleInner::ZenohShm(sbuf) => {
                        publisher
                            .put(sbuf)
                            .attachment(&metadata_bytes[..])
                            .wait()
                            .map_err(|e| eyre::eyre!("{e}"))
                            .wrap_err("zenoh SHM publish failed")?;
                    }
                    DataSampleInner::Vec(v) => {
                        publisher
                            .put(v.as_slice())
                            .attachment(&metadata_bytes[..])
                            .wait()
                            .map_err(|e| eyre::eyre!("{e}"))
                            .wrap_err("zenoh publish failed")?;
                    }
                }
            }
            // Data was (or would have been) sent via zenoh; daemon only needs the notification.
            None
        } else {
            // Fallback: send via control channel only (interactive/testing mode)
            sample.map(|s| s.finalize())
        };

        self.control_channel
            .send_message(output_id.clone(), metadata, data)
            .wrap_err_with(|| format!("failed to send output {output_id}"))?;

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

    /// Allocates a [`DataSample`] of the specified size.
    ///
    /// When a zenoh SHM provider is available, the buffer is allocated from shared memory,
    /// enabling zero-copy data transfer between nodes on the same machine.
    /// Falls back to an aligned Vec when no SHM provider is available (interactive/testing mode).
    ///
    /// # Blocking behavior
    ///
    /// SHM allocation uses `BlockOn<GarbageCollect>`: if the pool is full, it triggers
    /// garbage collection of released buffers and blocks until space is available. This
    /// can stall the sender if receivers hold onto buffers for a long time. Increase the
    /// pool size via the `DORA_NODE_SHM_POOL_SIZE` environment variable or
    /// `shared_memory_pool_size` in the dataflow YAML (default: 8 MB) if you observe
    /// send stalls.
    pub fn allocate_data_sample(&mut self, data_len: usize) -> eyre::Result<DataSample> {
        // Zero-length allocations are not supported by zenoh SHM; use a plain Vec.
        if data_len == 0 {
            let avec: AVec<u8, ConstAlign<128>> = AVec::__from_elem(128, 0, 0);
            return Ok(avec.into());
        }

        if let Some(provider) = &self.shm_provider {
            use zenoh::Wait;
            use zenoh::shm::{BlockOn, GarbageCollect};
            match provider
                .alloc(data_len)
                .with_policy::<BlockOn<GarbageCollect>>()
                .wait()
            {
                Ok(sbuf) => {
                    return Ok(DataSample {
                        len: data_len,
                        inner: DataSampleInner::ZenohShm(sbuf),
                    });
                }
                Err(e) => {
                    tracing::warn!("SHM allocation failed ({e}), falling back to heap buffer");
                }
            }
        }

        let avec: AVec<u8, ConstAlign<128>> = AVec::__from_elem(128, 0, data_len);
        Ok(avec.into())
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

        if let Err(err) = self.control_channel.report_outputs_done() {
            tracing::warn!("{err:?}")
        }
    }
}

/// A data region suitable for sending as an output message.
///
/// When zenoh SHM is available, the buffer is backed by shared memory for zero-copy transfer.
///
/// `DataSample` implements the [`Deref`] and [`DerefMut`] traits to read and write the mapped data.
pub struct DataSample {
    inner: DataSampleInner,
    len: usize,
}

impl DataSample {
    fn finalize(self) -> DataMessage {
        match self.inner {
            DataSampleInner::Vec(buffer) => DataMessage::Vec(buffer),
            DataSampleInner::ZenohShm(sbuf) => {
                // Copy SHM data into a Vec for the control channel fallback path
                let data = &sbuf[..self.len];
                DataMessage::Vec(AVec::from_slice(128, data))
            }
        }
    }
}

impl Deref for DataSample {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        match &self.inner {
            DataSampleInner::Vec(data) => &data[..self.len],
            DataSampleInner::ZenohShm(sbuf) => &sbuf[..self.len],
        }
    }
}

impl DerefMut for DataSample {
    fn deref_mut(&mut self) -> &mut Self::Target {
        match &mut self.inner {
            DataSampleInner::Vec(data) => &mut data[..self.len],
            DataSampleInner::ZenohShm(sbuf) => &mut sbuf[..self.len],
        }
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
            DataSampleInner::Vec(_) => "Vec",
            DataSampleInner::ZenohShm(_) => "ZenohShm",
        };
        f.debug_struct("DataSample")
            .field("len", &self.len)
            .field("kind", &kind)
            .finish_non_exhaustive()
    }
}

enum DataSampleInner {
    Vec(AVec<u8, ConstAlign<128>>),
    ZenohShm(zenoh::shm::ZShmMut),
}

/// Init Opentelemetry Tracing
///
/// This requires a tokio runtime spawning this function to be functional
#[cfg(feature = "tracing")]
pub fn init_tracing(
    node_id: &NodeId,
    dataflow_id: &DataflowId,
) -> eyre::Result<Arc<Mutex<Option<OtelGuard>>>> {
    let node_id_str = node_id.to_string();
    let guard: Arc<Mutex<Option<OtelGuard>>> = Arc::new(Mutex::new(None));
    let clone = guard.clone();
    let tracing_monitor = async move {
        let mut builder = TracingBuilder::new(node_id_str);
        // Only enable OTLP if environment variable is set
        if std::env::var("DORA_OTLP_ENDPOINT").is_ok()
            || std::env::var("DORA_JAEGER_TRACING").is_ok()
        {
            builder = builder
                .with_otlp_tracing()
                .context("failed to set up OTLP tracing")
                .unwrap()
                .with_stdout("info", true);
            *clone.lock().unwrap() = builder.guard.take();
        } else {
            builder = builder.with_stdout("info", true);
        }

        builder
            .build()
            .wrap_err("failed to set up tracing subscriber")
            .unwrap();
    };

    let rt = Handle::try_current().context("failed to get tokio runtime handle")?;
    rt.spawn(tracing_monitor);

    #[cfg(feature = "metrics")]
    {
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
    };
    Ok(guard)
}
