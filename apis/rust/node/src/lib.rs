pub use communication::Input;
use communication::STOP_TOPIC;
use communication_layer_pub_sub::CommunicationLayer;
pub use dora_core as core;
use dora_core::config::{CommunicationConfig, DataId, NodeId, NodeRunConfig};
pub use dora_message::Metadata;
use eyre::WrapErr;
pub use flume::Receiver;

pub mod communication;

pub struct DoraNode {
    id: NodeId,
    node_config: NodeRunConfig,
    communication: Box<dyn CommunicationLayer>,
}

impl DoraNode {
    pub fn init_from_env() -> eyre::Result<Self> {
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
        let communication_config = {
            let raw = std::env::var("DORA_COMMUNICATION_CONFIG")
                .wrap_err("env variable DORA_COMMUNICATION_CONFIG must be set")?;
            serde_yaml::from_str(&raw).context("failed to deserialize communication config")?
        };
        Self::init(id, node_config, communication_config)
    }

    pub fn init(
        id: NodeId,
        node_config: NodeRunConfig,
        communication_config: CommunicationConfig,
    ) -> eyre::Result<Self> {
        let communication = communication::init(&communication_config)?;
        Ok(Self {
            id,
            node_config,
            communication,
        })
    }

    pub fn inputs(&mut self) -> eyre::Result<flume::Receiver<Input>> {
        communication::subscribe_all(self.communication.as_mut(), &self.node_config.inputs)
    }

    pub fn send_output<F>(
        &mut self,
        output_id: &DataId,
        metadata: &Metadata,
        data_len: usize,
        data: F,
    ) -> eyre::Result<()>
    where
        F: FnOnce(&mut [u8]),
    {
        if !self.node_config.outputs.contains(output_id) {
            eyre::bail!("unknown output");
        }
        let serialized_metadata = metadata
            .serialize()
            .with_context(|| format!("failed to serialize `{}` message", output_id))?;
        let full_len = serialized_metadata.len() + data_len;

        let self_id = &self.id;
        let topic = format!("{self_id}/{output_id}");
        let publisher = self
            .communication
            .publisher(&topic)
            .map_err(|err| eyre::eyre!(err))
            .wrap_err_with(|| format!("failed create publisher for output {output_id}"))?;

        let mut sample = publisher
            .prepare(full_len)
            .map_err(|err| eyre::eyre!(err))?;
        let raw = sample.as_mut_slice();
        raw[..serialized_metadata.len()].copy_from_slice(&serialized_metadata);
        data(&mut raw[serialized_metadata.len()..]);
        sample
            .publish()
            .map_err(|err| eyre::eyre!(err))
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
        let self_id = &self.id;
        let topic = format!("{self_id}/{STOP_TOPIC}");
        let result = self
            .communication
            .publisher(&topic)
            .map_err(|err| eyre::eyre!(err))
            .wrap_err_with(|| {
                format!("failed to create publisher for stop message for node `{self_id}`")
            })
            .and_then(|p| {
                p.publish(&[])
                    .map_err(|err| eyre::eyre!(err))
                    .wrap_err_with(|| format!("failed to send stop message for node `{self_id}`"))
            });
        match result {
            Ok(()) => tracing::info!("sent stop message for {self_id}"),
            Err(err) => {
                tracing::error!("{err:?}")
            }
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
        let communication_config = config::CommunicationConfig::Zenoh {
            config: Default::default(),
            prefix: format!("/{}", uuid::Uuid::new_v4()),
        };

        let mut node = DoraNode::init(id, node_config, communication_config).unwrap();

        let inputs = node.inputs().unwrap();
        assert!(inputs.recv().is_err());
    }
}
