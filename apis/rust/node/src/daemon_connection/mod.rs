use crate::daemon_connection::interactive::InteractiveEvents;
use dora_core::{
    config::NodeId,
    uhlc::{self, Timestamp},
};
use dora_message::{
    DataflowId,
    common::{DropToken, Timestamped},
    daemon_to_node::{DaemonReply, NodeConfig, NodeDropEvent, NodeEvent, NodeEventOrUnknown},
    metadata::Metadata,
    node_to_daemon::{DaemonRequest, DataMessage, NodeControlClient, NodeRegisterRequest},
    tarpc::{self, client, serde_transport},
};
use eyre::{Context, bail, eyre};
pub use node_integration_testing::IntegrationTestingEvents;
use std::{
    net::SocketAddr,
    sync::Arc,
    time::{Duration, Instant},
};
use tokio::sync::oneshot;

mod interactive;
pub(crate) mod node_integration_testing;

mod json_to_arrow;

pub enum DaemonChannel {
    Tarpc {
        client: NodeControlClient,
        /// Dedicated thread + runtime for tarpc I/O.
        ///
        /// All `block_on` calls are dispatched to this thread so that
        /// `DaemonChannel` can be used from within an existing tokio
        /// runtime (e.g. Python operators inside `dora.start_runtime()`).
        rt: TarpcRuntime,
        clock: Arc<uhlc::HLC>,
    },
    Interactive(InteractiveEvents),
    IntegrationTestChannel(
        tokio::sync::mpsc::Sender<(
            Timestamped<DaemonRequest>,
            tokio::sync::oneshot::Sender<DaemonReply>,
        )>,
    ),
}

/// A tokio runtime that lives on its own OS thread so that `block_on`
/// never conflicts with an outer runtime on the calling thread.
pub struct TarpcRuntime {
    tx: std::sync::mpsc::Sender<Box<dyn FnOnce(&tokio::runtime::Runtime) + Send>>,
}

impl TarpcRuntime {
    fn new() -> eyre::Result<Self> {
        let (tx, rx) =
            std::sync::mpsc::channel::<Box<dyn FnOnce(&tokio::runtime::Runtime) + Send>>();
        std::thread::spawn(move || {
            let runtime = tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
                .expect("failed to create tokio runtime for tarpc client");
            while let Ok(task) = rx.recv() {
                task(&runtime);
            }
        });
        Ok(Self { tx })
    }

    /// Run a closure on the dedicated runtime thread and block the
    /// calling thread until it completes.
    fn block_on<T: Send + 'static>(
        &self,
        task: impl FnOnce(&tokio::runtime::Runtime) -> T + Send + 'static,
    ) -> T {
        let (result_tx, result_rx) = std::sync::mpsc::channel();
        let _ = self.tx.send(Box::new(move |rt| {
            let _ = result_tx.send(task(rt));
        }));
        result_rx
            .recv()
            .expect("tarpc runtime thread dropped unexpectedly")
    }
}

/// Long deadline for tarpc RPC calls (10 minutes).
/// next_event and next_finished_drop_tokens can block for a long time.
fn long_context() -> tarpc::context::Context {
    let mut ctx = tarpc::context::current();
    ctx.deadline = Instant::now() + Duration::from_secs(600);
    ctx
}

impl DaemonChannel {
    #[tracing::instrument(level = "trace", skip(clock))]
    pub fn new_tcp(socket_addr: SocketAddr, clock: Arc<uhlc::HLC>) -> eyre::Result<Self> {
        let rt = TarpcRuntime::new()?;

        let client = rt.block_on(move |rt| {
            rt.block_on(async {
                let transport = serde_transport::tcp::connect(
                    socket_addr,
                    tarpc::tokio_serde::formats::Bincode::default,
                )
                .await?;
                Ok::<_, eyre::Error>(
                    NodeControlClient::new(client::Config::default(), transport).spawn(),
                )
            })
        })?;

        Ok(DaemonChannel::Tarpc { client, rt, clock })
    }

    pub fn register(
        &mut self,
        dataflow_id: DataflowId,
        node_id: NodeId,
        timestamp: Timestamp,
    ) -> eyre::Result<()> {
        match self {
            DaemonChannel::Tarpc { client, rt, clock } => {
                let client = client.clone();
                let request = NodeRegisterRequest::new(dataflow_id, node_id);
                let ts = clock.new_timestamp();
                rt.block_on(move |rt| rt.block_on(client.register(long_context(), ts, request)))
                    .map_err(|e| eyre!("{e}"))?
                    .map_err(|e| eyre!("{e}"))
                    .wrap_err("failed to register node with dora-daemon")
            }
            DaemonChannel::Interactive(_) => Ok(()),
            DaemonChannel::IntegrationTestChannel(channel) => {
                let msg = Timestamped {
                    inner: DaemonRequest::Register(NodeRegisterRequest::new(dataflow_id, node_id)),
                    timestamp,
                };
                let (reply_tx, reply) = oneshot::channel();
                channel
                    .blocking_send((msg, reply_tx))
                    .expect("failed to send request to IntegrationTestChannel");
                let reply = reply
                    .blocking_recv()
                    .context("failed to receive oneshot reply")?;
                match reply {
                    DaemonReply::Result(result) => result
                        .map_err(|e| eyre!(e))
                        .wrap_err("failed to register node with dora-daemon"),
                    other => bail!("unexpected register reply: {other:?}"),
                }
            }
        }
    }

    pub fn subscribe(&mut self) -> eyre::Result<()> {
        match self {
            DaemonChannel::Tarpc { client, rt, clock } => {
                let client = client.clone();
                let ts = clock.new_timestamp();
                rt.block_on(move |rt| rt.block_on(client.subscribe(long_context(), ts)))
                    .map_err(|e| eyre!("{e}"))?
                    .map_err(|e| eyre!("{e}"))
            }
            DaemonChannel::Interactive(_) => Ok(()),
            DaemonChannel::IntegrationTestChannel(_) => {
                // handled via legacy request path
                Ok(())
            }
        }
    }

    pub fn subscribe_drop(&mut self) -> eyre::Result<()> {
        match self {
            DaemonChannel::Tarpc { client, rt, clock } => {
                let client = client.clone();
                let ts = clock.new_timestamp();
                rt.block_on(move |rt| rt.block_on(client.subscribe_drop(long_context(), ts)))
                    .map_err(|e| eyre!("{e}"))?
                    .map_err(|e| eyre!("{e}"))
            }
            DaemonChannel::Interactive(_) => Ok(()),
            DaemonChannel::IntegrationTestChannel(_) => Ok(()),
        }
    }

    /// Returns `Some(events)` on real data, `None` as a keepalive (caller should retry).
    pub fn next_event(
        &mut self,
        drop_tokens: Vec<DropToken>,
    ) -> eyre::Result<Option<Vec<Timestamped<NodeEventOrUnknown>>>> {
        match self {
            DaemonChannel::Tarpc { client, rt, clock } => {
                let client = client.clone();
                let ts = clock.new_timestamp();
                rt.block_on(move |rt| {
                    rt.block_on(client.next_event(long_context(), ts, drop_tokens))
                })
                .map_err(|e| eyre!("{e}"))
                .map(|opt| opt.map(|events| events.into_iter().map(Into::into).collect()))
            }
            _ => {
                let reply = self.request(&Timestamped {
                    inner: DaemonRequest::NextEvent { drop_tokens },
                    timestamp: dora_core::uhlc::HLC::default().new_timestamp(),
                })?;
                match reply {
                    DaemonReply::NextEvents(events) => Ok(Some(events)),
                    DaemonReply::Result(Err(err)) => Err(eyre!("{err}")),
                    other => bail!("unexpected NextEvent reply: {other:?}"),
                }
            }
        }
    }

    /// Returns `Some(events)` on real data, `None` as a keepalive (caller should retry).
    pub fn next_finished_drop_tokens(
        &mut self,
    ) -> eyre::Result<Option<Vec<Timestamped<NodeDropEvent>>>> {
        match self {
            DaemonChannel::Tarpc { client, rt, clock } => {
                let client = client.clone();
                let ts = clock.new_timestamp();
                rt.block_on(move |rt| {
                    rt.block_on(client.next_finished_drop_tokens(long_context(), ts))
                })
                .map_err(|e| eyre!("{e}"))
            }
            _ => {
                let reply = self.request(&Timestamped {
                    inner: DaemonRequest::NextFinishedDropTokens,
                    timestamp: dora_core::uhlc::HLC::default().new_timestamp(),
                })?;
                match reply {
                    DaemonReply::NextDropEvents(events) => Ok(Some(events)),
                    other => bail!("unexpected NextFinishedDropTokens reply: {other:?}"),
                }
            }
        }
    }

    pub fn send_message(
        &mut self,
        output_id: dora_core::config::DataId,
        metadata: Metadata,
        data: Option<DataMessage>,
    ) -> eyre::Result<()> {
        match self {
            DaemonChannel::Tarpc { client, rt, clock } => {
                let client = client.clone();
                let ts = clock.new_timestamp();
                rt.block_on(move |rt| {
                    rt.block_on(client.send_message(long_context(), ts, output_id, metadata, data))
                })
                .map_err(|e| eyre!("{e}"))?
                .map_err(|e| eyre!("{e}"))
            }
            _ => {
                let _reply = self.request(&Timestamped {
                    inner: DaemonRequest::SendMessage {
                        output_id,
                        metadata,
                        data,
                    },
                    timestamp: dora_core::uhlc::HLC::default().new_timestamp(),
                })?;
                Ok(())
            }
        }
    }

    pub fn close_outputs(&mut self, outputs: Vec<dora_core::config::DataId>) -> eyre::Result<()> {
        match self {
            DaemonChannel::Tarpc { client, rt, clock } => {
                let client = client.clone();
                let ts = clock.new_timestamp();
                rt.block_on(move |rt| {
                    rt.block_on(client.close_outputs(long_context(), ts, outputs))
                })
                .map_err(|e| eyre!("{e}"))?
                .map_err(|e| eyre!("{e}"))
            }
            _ => {
                let reply = self.request(&Timestamped {
                    inner: DaemonRequest::CloseOutputs(outputs),
                    timestamp: dora_core::uhlc::HLC::default().new_timestamp(),
                })?;
                match reply {
                    DaemonReply::Result(r) => r.map_err(|e| eyre!("{e}")),
                    other => bail!("unexpected CloseOutputs reply: {other:?}"),
                }
            }
        }
    }

    pub fn outputs_done(&mut self) -> eyre::Result<()> {
        match self {
            DaemonChannel::Tarpc { client, rt, clock } => {
                let client = client.clone();
                let ts = clock.new_timestamp();
                rt.block_on(move |rt| rt.block_on(client.outputs_done(long_context(), ts)))
                    .map_err(|e| eyre!("{e}"))?
                    .map_err(|e| eyre!("{e}"))
            }
            _ => {
                let reply = self.request(&Timestamped {
                    inner: DaemonRequest::OutputsDone,
                    timestamp: dora_core::uhlc::HLC::default().new_timestamp(),
                })?;
                match reply {
                    DaemonReply::Result(r) => r.map_err(|e| eyre!("{e}")),
                    other => bail!("unexpected OutputsDone reply: {other:?}"),
                }
            }
        }
    }

    pub fn report_drop_tokens_rpc(&mut self, drop_tokens: Vec<DropToken>) -> eyre::Result<()> {
        match self {
            DaemonChannel::Tarpc { client, rt, clock } => {
                let client = client.clone();
                let ts = clock.new_timestamp();
                rt.block_on(move |rt| {
                    rt.block_on(client.report_drop_tokens(long_context(), ts, drop_tokens))
                })
                .map_err(|e| eyre!("{e}"))?
                .map_err(|e| eyre!("{e}"))
            }
            _ => {
                let _reply = self.request(&Timestamped {
                    inner: DaemonRequest::ReportDropTokens { drop_tokens },
                    timestamp: dora_core::uhlc::HLC::default().new_timestamp(),
                })?;
                Ok(())
            }
        }
    }

    pub fn event_stream_dropped(&mut self) -> eyre::Result<()> {
        match self {
            DaemonChannel::Tarpc { client, rt, clock } => {
                let client = client.clone();
                let ts = clock.new_timestamp();
                rt.block_on(move |rt| rt.block_on(client.event_stream_dropped(long_context(), ts)))
                    .map_err(|e| eyre!("{e}"))?
                    .map_err(|e| eyre!("{e}"))
            }
            _ => {
                let reply = self.request(&Timestamped {
                    inner: DaemonRequest::EventStreamDropped,
                    timestamp: dora_core::uhlc::HLC::default().new_timestamp(),
                })?;
                match reply {
                    DaemonReply::Result(r) => r.map_err(|e| eyre!("{e}")),
                    other => bail!("unexpected EventStreamDropped reply: {other:?}"),
                }
            }
        }
    }

    pub fn node_config_rpc(&mut self, node_id: NodeId) -> eyre::Result<NodeConfig> {
        match self {
            DaemonChannel::Tarpc { client, rt, clock } => {
                let client = client.clone();
                let ts = clock.new_timestamp();
                let yaml = rt
                    .block_on(move |rt| {
                        rt.block_on(client.node_config(long_context(), ts, node_id))
                    })
                    .map_err(|e| eyre!("{e}"))?
                    .map_err(|e| eyre!("{e}"))?;
                serde_yaml::from_str(&yaml).context("failed to deserialize node config from daemon")
            }
            _ => {
                let reply = self.request(&Timestamped {
                    inner: DaemonRequest::NodeConfig { node_id },
                    timestamp: dora_core::uhlc::HLC::default().new_timestamp(),
                })?;
                match reply {
                    DaemonReply::NodeConfig { result } => result.map_err(|e| eyre!("{e}")),
                    other => bail!("unexpected NodeConfig reply: {other:?}"),
                }
            }
        }
    }

    /// Legacy request method for Interactive and IntegrationTestChannel variants.
    pub fn request(&mut self, request: &Timestamped<DaemonRequest>) -> eyre::Result<DaemonReply> {
        match self {
            DaemonChannel::Tarpc { .. } => {
                bail!("request() should not be called on Tarpc channels; use typed methods instead")
            }
            DaemonChannel::Interactive(events) => events.request(request),
            DaemonChannel::IntegrationTestChannel(channel) => {
                let (reply_tx, reply) = oneshot::channel();
                channel
                    .blocking_send((request.clone(), reply_tx))
                    .expect("failed to send request to IntegrationTestChannel");
                reply
                    .blocking_recv()
                    .context("failed to receive oneshot reply")
            }
        }
    }
}
