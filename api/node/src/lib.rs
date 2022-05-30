use communication::CommunicationLayer;
use config::{CommunicationConfig, DataId, NodeId, NodeRunConfig};
use eyre::WrapErr;
use futures::{stream::FuturesUnordered, StreamExt};
use futures_concurrency::Merge;
use std::{collections::HashSet, time::Duration};
pub mod communication;
pub mod config;

#[doc(hidden)]
pub const STOP_TOPIC: &str = "__dora_rs_internal__operator_stopped";
use paho_mqtt as mqtt;

pub struct DoraNode {
    id: NodeId,
    node_config: NodeRunConfig,
    communication_config: CommunicationConfig,
    communication: Box<dyn CommunicationLayer>,
}

impl DoraNode {
    pub async fn init_from_env() -> eyre::Result<Self> {
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
        Self::init_mqtt(id, node_config, communication_config).await
    }

    pub async fn init_zenoh(
        id: NodeId,
        node_config: NodeRunConfig,
        communication_config: CommunicationConfig,
    ) -> eyre::Result<Self> {
        let zenoh = zenoh::open(communication_config.zenoh_config.clone())
            .await
            .map_err(BoxError)
            .wrap_err("failed to create zenoh session")?;

        Ok(Self {
            id,
            node_config,
            communication_config,
            communication: Box::new(zenoh),
        })
    }

    pub async fn init_mqtt(
        id: NodeId,
        node_config: NodeRunConfig,
        communication_config: CommunicationConfig,
    ) -> eyre::Result<Self> {
        let host_config = "tcp://localhost:1883";
        // Create the client. Use an ID for a persistent session.
        // A real system should try harder to use a unique ID.

        let create_opts = mqtt::CreateOptionsBuilder::new()
            .server_uri(host_config)
            .client_id(format!("mqtt_client_{}", id))
            .finalize();
        // Create the client connection
        let client = mqtt::AsyncClient::new(create_opts).unwrap();

        // Define the set of options for the connection
        let lwt = mqtt::Message::new("test", "Async subscriber lost connection", mqtt::QOS_1);

        let conn_opts = mqtt::ConnectOptionsBuilder::new()
            .keep_alive_interval(Duration::from_secs(10))
            .mqtt_version(mqtt::MQTT_VERSION_3_1_1)
            .clean_session(false)
            .will_message(lwt)
            .finalize();
        client.connect(conn_opts).await?;
        Ok(Self {
            id,
            node_config,
            communication_config,
            communication: Box::new(client),
        })
    }

    pub async fn inputs(&self) -> eyre::Result<impl futures::Stream<Item = Input> + '_> {
        let prefix = &self.communication_config.zenoh_prefix;

        let mut streams = Vec::new();
        for (
            input,
            config::InputMapping {
                source,
                operator,
                output,
            },
        ) in &self.node_config.inputs
        {
            let topic = match operator {
                Some(operator) => format!("{prefix}/{source}/{operator}/{output}"),
                None => format!("{prefix}/{source}/{output}"),
            };
            let sub = self
                .communication
                .subscribe(&topic)
                .await
                .wrap_err_with(|| format!("failed to subscribe on {topic}"))?;
            streams.push(sub.map(|data| Input {
                id: input.clone(),
                data,
            }))
        }

        //let stop_messages = FuturesUnordered::new();
        //let sources: HashSet<_> = self
        //.node_config
        //.inputs
        //.values()
        //.map(|v| (&v.source, &v.operator))
        //.collect();
        //for (source, operator) in &sources {
        //let topic = match operator {
        //Some(operator) => format!("{prefix}/{source}/{operator}/{STOP_TOPIC}"),
        //None => format!("{prefix}/{source}/{STOP_TOPIC}"),
        //};
        //let sub = self
        //.communication
        //.subscribe(&topic)
        //.await
        //.wrap_err_with(|| format!("failed to subscribe on {topic}"))?;
        //stop_messages.push(sub.into_future());
        //}
        //let finished = Box::pin(stop_messages.all(|_| async { true }));

        Ok(streams.merge()) //.take_until(finished))
    }

    pub async fn send_output(&self, output_id: &DataId, data: &[u8]) -> eyre::Result<()> {
        if !self.node_config.outputs.contains(output_id) {
            eyre::bail!("unknown output");
        }

        let prefix = &self.communication_config.zenoh_prefix;
        let self_id = &self.id;

        let topic = format!("{prefix}/{self_id}/{output_id}");
        self.communication
            .publish(&topic, data)
            .await
            .wrap_err_with(|| format!("failed to send data for output {output_id}"))?;
        Ok(())
    }
}

impl Drop for DoraNode {
    fn drop(&mut self) {
        let prefix = &self.communication_config.zenoh_prefix;
        let self_id = &self.id;
        let topic = format!("{prefix}/{self_id}/{STOP_TOPIC}");
        let result = self
            .communication
            .publish_sync(&topic, &[])
            .wrap_err_with(|| format!("failed to send stop message for source `{self_id}`"));
        if let Err(err) = result {
            tracing::error!("{err}")
        }
    }
}

pub struct Input {
    pub id: DataId,
    pub data: Vec<u8>,
}

pub struct BoxError(Box<dyn std::error::Error + Send + Sync + 'static>);

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

    fn run<F, O>(future: F) -> O
    where
        F: std::future::Future<Output = O>,
    {
        let rt = tokio::runtime::Builder::new_current_thread()
            .build()
            .unwrap();
        rt.block_on(future)
    }

    #[test]
    fn no_op_operator() {
        let id = uuid::Uuid::new_v4().to_string().into();
        let node_config = config::NodeRunConfig {
            inputs: Default::default(),
            outputs: Default::default(),
        };
        let communication_config = config::CommunicationConfig {
            zenoh_config: Default::default(),
            zenoh_prefix: format!("/{}", uuid::Uuid::new_v4()),
        };

        run(async {
            let operator = DoraNode::init_mqtt(id, node_config, communication_config)
                .await
                .unwrap();
            let mut inputs = operator.inputs().await.unwrap();
            assert!(inputs.next().await.is_none());
        });
    }
}
