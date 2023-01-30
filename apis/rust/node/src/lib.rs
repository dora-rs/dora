use std::thread::JoinHandle;

use daemon::{ControlChannel, DaemonConnection, EventStream};
pub use dora_core;
use dora_core::{
    config::{DataId, NodeId, NodeRunConfig},
    daemon_messages::NodeConfig,
};
pub use dora_message::{uhlc, Metadata, MetadataParameters};
use eyre::WrapErr;
pub use flume::Receiver;
use shared_memory_server::ShmemConf;

pub mod daemon;

pub struct DoraNode {
    id: NodeId,
    node_config: NodeRunConfig,
    control_channel: ControlChannel,
    hlc: uhlc::HLC,
    event_stream_thread: Option<JoinHandle<()>>,
}

impl DoraNode {
    pub fn init_from_env() -> eyre::Result<(Self, EventStream)> {
        #[cfg(feature = "tracing-subscriber")]
        set_up_tracing().context("failed to set up tracing subscriber")?;

        let node_config = {
            let raw = std::env::var("DORA_NODE_CONFIG")
                .wrap_err("env variable DORA_NODE_CONFIG must be set")?;
            serde_yaml::from_str(&raw).context("failed to deserialize operator config")?
        };
        Self::init(node_config)
    }

    pub fn init(node_config: NodeConfig) -> eyre::Result<(Self, EventStream)> {
        let NodeConfig {
            dataflow_id,
            node_id,
            run_config,
            daemon_control_region_id,
            daemon_events_region_id,
        } = node_config;

        let DaemonConnection {
            control_channel,
            event_stream,
            event_stream_thread,
        } = DaemonConnection::init(
            dataflow_id,
            &node_id,
            &daemon_control_region_id,
            &daemon_events_region_id,
        )
        .wrap_err("failed to connect to dora-daemon")?;

        let node = Self {
            id: node_id,
            node_config: run_config,
            control_channel,
            hlc: uhlc::HLC::default(),
            event_stream_thread: Some(event_stream_thread),
        };
        Ok((node, event_stream))
    }

    pub fn send_output<F>(
        &mut self,
        output_id: DataId,
        parameters: MetadataParameters,
        data_len: usize,
        data: F,
    ) -> eyre::Result<()>
    where
        F: FnOnce(&mut [u8]),
    {
        if !self.node_config.outputs.contains(&output_id) {
            eyre::bail!("unknown output");
        }
        let metadata = Metadata::from_parameters(self.hlc.new_timestamp(), parameters.into_owned());

        if data_len > 0 {
            let sample = self
                .control_channel
                .prepare_message(output_id.clone(), metadata, data_len)
                .wrap_err("failed to prepare sample for output message")?;
            // map shared memory and fill in data
            let mut shared_memory = ShmemConf::new()
                .os_id(&sample.id)
                .open()
                .wrap_err("failed to open shared memory sample")?;

            let raw = unsafe { shared_memory.as_slice_mut() };
            data(&mut raw[..data_len]);

            self.control_channel
                .send_prepared_message(sample)
                .wrap_err_with(|| format!("failed to send data for output {output_id}"))?;
        } else {
            data(&mut []);
            self.control_channel
                .send_empty_message(output_id.clone(), metadata)
                .wrap_err_with(|| format!("failed to send output {output_id}"))?;
        }

        Ok(())
    }

    pub fn close_outputs(&mut self, outputs: Vec<DataId>) -> eyre::Result<()> {
        for output_id in &outputs {
            if !self.node_config.outputs.remove(output_id) {
                eyre::bail!("unknown output {output_id}");
            }
        }

        self.control_channel
            .report_closed_outputs(outputs)
            .wrap_err("failed to report closed outputs to daemon")?;

        Ok(())
    }

    pub fn id(&self) -> &NodeId {
        &self.id
    }

    pub fn node_config(&self) -> &NodeRunConfig {
        &self.node_config
    }
}

impl Drop for DoraNode {
    #[tracing::instrument(skip(self), fields(self.id = %self.id))]
    fn drop(&mut self) {
        match self.control_channel.report_stop() {
            Ok(()) => {
                if let Some(thread) = self.event_stream_thread.take() {
                    if let Err(panic) = thread.join() {
                        std::panic::resume_unwind(panic);
                    }
                }
            }
            Err(err) => tracing::error!("{err:?}"),
        }
    }
}

pub type BoxError = Box<dyn std::error::Error + Send + Sync + 'static>;

#[cfg(feature = "tracing-subscriber")]
fn set_up_tracing() -> eyre::Result<()> {
    use tracing_subscriber::prelude::__tracing_subscriber_SubscriberExt;

    let stdout_log = tracing_subscriber::fmt::layer().pretty();
    let subscriber = tracing_subscriber::Registry::default().with(stdout_log);
    tracing::subscriber::set_global_default(subscriber)
        .context("failed to set tracing global subscriber")
}
