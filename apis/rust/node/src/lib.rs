pub use communication::Input;
use communication::{CommunicationLayer, STOP_TOPIC};
use config::{CommunicationConfig, DataId, NodeId, NodeRunConfig};
use eyre::WrapErr;

pub mod communication;
pub mod config;

pub struct DoraNode {
    id: NodeId,
    node_config: NodeRunConfig,
    communication: Box<dyn CommunicationLayer>,
}

impl DoraNode {
    pub fn init_from_env() -> eyre::Result<Self> {
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
        self.communication.subscribe_all(&self.node_config.inputs)
    }

    pub fn send_output(&mut self, output_id: &DataId, data: &[u8]) -> eyre::Result<()> {
        if !self.node_config.outputs.contains(output_id) {
            eyre::bail!("unknown output");
        }

        let self_id = &self.id;

        let topic = format!("{self_id}/{output_id}");
        self.communication
            .publisher(&topic)
            .wrap_err_with(|| format!("failed create publisher for output {output_id}"))?
            .publish(data)
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
    fn drop(&mut self) {
        let self_id = &self.id;
        let topic = format!("{self_id}/{STOP_TOPIC}");
        let result = self
            .communication
            .publisher(&topic)
            .wrap_err_with(|| {
                format!("failed to create publisher for stop message for node `{self_id}`")
            })
            .and_then(|p| {
                p.publish(&[])
                    .wrap_err_with(|| format!("failed to send stop message for node `{self_id}`"))
            });
        match result {
            Ok(()) => println!("sent stop message for {self_id}"),
            Err(err) => {
                println!("error sending stop message for {self_id}: {err:?}");
                tracing::error!("{err:?}")
            }
        }
    }
}

pub struct BoxError(pub Box<dyn std::error::Error + Send + Sync + 'static>);

impl std::fmt::Debug for BoxError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        std::fmt::Debug::fmt(&self.0, f)
    }
}

impl std::fmt::Display for BoxError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        std::fmt::Display::fmt(&self.0, f)
    }
}

impl std::error::Error for BoxError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        self.0.source()
    }
}

#[cfg(test)]
mod tests {
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
