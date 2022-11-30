use daemon::{ControlChannel, DaemonConnection, EventStream};
pub use dora_core;
use dora_core::config::{DataId, NodeId, NodeRunConfig};
pub use dora_message::{uhlc, Metadata, MetadataParameters};
use eyre::WrapErr;
pub use flume::Receiver;

pub mod daemon;

pub struct DoraNode {
    id: NodeId,
    node_config: NodeRunConfig,
    control_channel: ControlChannel,
    hlc: uhlc::HLC,
}

impl DoraNode {
    pub fn init_from_env() -> eyre::Result<(Self, EventStream)> {
        #[cfg(feature = "tracing-subscriber")]
        set_up_tracing().context("failed to set up tracing subscriber")?;

        let id = {
            let raw =
                std::env::var("DORA_NODE_ID").wrap_err("env variable DORA_NODE_ID must be set")?;
            serde_yaml::from_str(&raw).context("failed to deserialize operator config")?
        };
        let node_config = {
            let raw = std::env::var("DORA_NODE_RUN_CONFIG")
                .wrap_err("env variable DORA_NODE_RUN_CONFIG must be set")?;
            serde_yaml::from_str(&raw).context("failed to deserialize operator config")?
        };
        Self::init(id, node_config)
    }

    pub fn init(id: NodeId, node_config: NodeRunConfig) -> eyre::Result<(Self, EventStream)> {
        let DaemonConnection {
            control_channel,
            event_stream,
        } = DaemonConnection::init(id.clone()).wrap_err("failed to connect to dora-daemon")?;

        let node = Self {
            id,
            node_config,
            control_channel,
            hlc: uhlc::HLC::default(),
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
        let metadata = Metadata::from_parameters(self.hlc.new_timestamp(), parameters);
        let serialized_metadata = metadata
            .serialize()
            .with_context(|| format!("failed to serialize `{}` message", output_id))?;
        let full_len = serialized_metadata.len() + data_len;

        let sample = self
            .control_channel
            .prepare_message(output_id.clone(), full_len)
            .wrap_err("failed to prepare sample for output message")?;

        let raw = sample.data.get_mut();
        raw[..serialized_metadata.len()].copy_from_slice(&serialized_metadata);
        data(&mut raw[serialized_metadata.len()..]);

        self.control_channel
            .send_message(sample)
            .wrap_err_with(|| format!("failed to send data for output {output_id}"))?;
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
        if let Err(err) = self.control_channel.report_stop() {
            tracing::error!("{err:?}");
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

#[cfg(test)]
mod tests {
    use dora_core::config;

    use super::*;

    #[test]
    fn no_op_operator() {
        let id = uuid::Uuid::new_v4().to_string().into();
        let node_config = config::NodeRunConfig {
            inputs: Default::default(),
            outputs: Default::default(),
        };

        let (_node, events) = DoraNode::init(id, node_config).unwrap();

        assert!(events.recv().is_err());
    }
}
