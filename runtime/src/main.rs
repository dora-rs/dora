#![warn(unsafe_op_in_unsafe_fn)]

use dora_api::{
    self,
    communication::CommunicationLayer,
    config::{CommunicationConfig, DataId, InputMapping, NodeId, OperatorId},
    STOP_TOPIC,
};
use dora_common::{descriptor::OperatorConfig, BoxError};
use eyre::{bail, eyre, Context};
use futures::{stream::FuturesUnordered, StreamExt};
use futures_concurrency::Merge;
use operator::{Operator, OperatorEvent};
use std::collections::{BTreeMap, HashSet};
use tokio::sync::mpsc;
use tokio_stream::{wrappers::ReceiverStream, StreamMap};

mod operator;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    let node_id = {
        let raw =
            std::env::var("DORA_NODE_ID").wrap_err("env variable DORA_NODE_ID must be set")?;
        serde_yaml::from_str(&raw).context("failed to deserialize operator config")?
    };
    let communication_config: CommunicationConfig = {
        let raw = std::env::var("DORA_COMMUNICATION_CONFIG")
            .wrap_err("env variable DORA_COMMUNICATION_CONFIG must be set")?;
        serde_yaml::from_str(&raw).context("failed to deserialize communication config")?
    };
    let operators: Vec<OperatorConfig> = {
        let raw =
            std::env::var("DORA_OPERATORS").wrap_err("env variable DORA_OPERATORS must be set")?;
        serde_yaml::from_str(&raw).context("failed to deserialize operator config")?
    };

    let mut operator_map = BTreeMap::new();
    let mut operator_events = StreamMap::new();
    for operator_config in &operators {
        let (events_tx, events) = mpsc::channel(1);
        let operator = Operator::init(operator_config.clone(), events_tx.clone())
            .await
            .wrap_err_with(|| format!("failed to init operator {}", operator_config.id))?;
        operator_map.insert(&operator_config.id, operator);
        operator_events.insert(operator_config.id.clone(), ReceiverStream::new(events));
    }

    let zenoh = zenoh::open(communication_config.zenoh_config.clone())
        .await
        .map_err(BoxError)
        .wrap_err("failed to create zenoh session")?;
    let communication: Box<dyn CommunicationLayer> = Box::new(zenoh);

    let inputs = subscribe(communication.as_ref(), &communication_config, &operators)
        .await
        .context("failed to subscribe")?;

    let input_events = inputs.map(Event::Input);
    let operator_events = operator_events.map(|(id, event)| Event::Operator { id, event });
    let mut events = (input_events, operator_events).merge();

    while let Some(event) = events.next().await {
        match event {
            Event::Input(input) => {
                let operator = operator_map
                    .get_mut(&input.target_operator)
                    .ok_or_else(|| {
                        eyre!(
                            "received input for unexpected operator `{}`",
                            input.target_operator
                        )
                    })?;

                println!(
                    "Received input {} for operator {}: {}",
                    input.id,
                    input.target_operator,
                    String::from_utf8_lossy(&input.data)
                );

                operator
                    .handle_input(input.id.clone(), input.data)
                    .wrap_err_with(|| {
                        format!(
                            "operator {} failed to handle input {}",
                            input.target_operator, input.id
                        )
                    })?;
            }
            Event::Operator { id, event } => {
                let operator = operator_map
                    .get(&id)
                    .ok_or_else(|| eyre!("received event from unknown operator {id}"))?;
                match event {
                    OperatorEvent::Output { id: data_id, value } => {
                        if !operator.config().outputs.contains(&data_id) {
                            eyre::bail!("unknown output {data_id} for operator {id}");
                        }
                        publish(
                            &node_id,
                            id,
                            data_id,
                            &value,
                            communication.as_ref(),
                            &communication_config,
                        )
                        .await
                        .context("failed to publish operator output")?;
                    }
                    OperatorEvent::Error(err) => {
                        bail!(err.wrap_err(format!("operator {id} failed")))
                    }
                    OperatorEvent::Panic(payload) => std::panic::resume_unwind(payload),
                }
            }
        }
    }

    Ok(())
}

async fn subscribe<'a>(
    communication: &'a dyn CommunicationLayer,
    communication_config: &CommunicationConfig,
    operators: &'a [OperatorConfig],
) -> eyre::Result<impl futures::Stream<Item = OperatorInput> + 'a> {
    let prefix = &communication_config.zenoh_prefix;

    let mut streams = Vec::new();

    for operator in operators {
        for (input, mapping) in &operator.inputs {
            let InputMapping {
                source,
                operator: source_operator,
                output,
            } = mapping;
            let topic = match source_operator {
                Some(operator) => format!("{prefix}/{source}/{operator}/{output}"),
                None => format!("{prefix}/{source}/{output}"),
            };
            println!("subscribing to {topic}");
            let sub = communication
                .subscribe(&topic)
                .await
                .wrap_err_with(|| format!("failed to subscribe on {topic}"))?;
            streams.push(sub.map(|data| OperatorInput {
                target_operator: operator.id.clone(),
                id: input.clone(),
                data,
            }))
        }
    }
    let stop_messages = FuturesUnordered::new();
    let sources: HashSet<_> = operators
        .iter()
        .flat_map(|o| o.inputs.values())
        .map(|v| (&v.source, &v.operator))
        .collect();
    for (source, operator) in &sources {
        let topic = match operator {
            Some(operator) => format!("{prefix}/{source}/{operator}/{STOP_TOPIC}"),
            None => format!("{prefix}/{source}/{STOP_TOPIC}"),
        };
        println!("subscribing to {topic}");
        let sub = communication
            .subscribe(&topic)
            .await
            .wrap_err_with(|| format!("failed to subscribe on {topic}"))?;
        stop_messages.push(sub.into_future());
    }
    let finished = Box::pin(stop_messages.all(|_| async { true }));

    Ok(streams.merge().take_until(finished))
}

async fn publish(
    self_id: &NodeId,
    operator_id: OperatorId,
    output_id: DataId,
    value: &[u8],
    communication: &dyn CommunicationLayer,
    communication_config: &CommunicationConfig,
) -> eyre::Result<()> {
    let prefix = &communication_config.zenoh_prefix;

    let topic = format!("{prefix}/{self_id}/{operator_id}/{output_id}");
    communication
        .publish(&topic, value)
        .await
        .wrap_err_with(|| format!("failed to send data for output {output_id}"))?;

    Ok(())
}

enum Event {
    Input(OperatorInput),
    Operator {
        id: OperatorId,
        event: OperatorEvent,
    },
}

struct OperatorInput {
    pub target_operator: OperatorId,
    pub id: DataId,
    pub data: Vec<u8>,
}

/*
pub struct DoraNode {
    id: NodeId,
    operator_config: NodeRunConfig,
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
        let operator_config = {
            let raw = std::env::var("DORA_NODE_RUN_CONFIG")
                .wrap_err("env variable DORA_NODE_RUN_CONFIG must be set")?;
            serde_yaml::from_str(&raw).context("failed to deserialize operator config")?
        };
        let communication_config = {
            let raw = std::env::var("DORA_COMMUNICATION_CONFIG")
                .wrap_err("env variable DORA_COMMUNICATION_CONFIG must be set")?;
            serde_yaml::from_str(&raw).context("failed to deserialize communication config")?
        };
        Self::init(id, operator_config, communication_config).await
    }

    pub async fn init(
        id: NodeId,
        operator_config: NodeRunConfig,
        communication_config: CommunicationConfig,
    ) -> eyre::Result<Self> {
        let zenoh = zenoh::open(communication_config.zenoh_config.clone())
            .await
            .map_err(BoxError)
            .wrap_err("failed to create zenoh session")?;

        Ok(Self {
            id,
            operator_config,
            communication_config,
            communication: Box::new(zenoh),
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
        ) in &self.operator_config.inputs
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

        let stop_messages = FuturesUnordered::new();
        let sources: HashSet<_> = self
            .operator_config
            .inputs
            .values()
            .map(|v| &v.source)
            .collect();
        for source in &sources {
            let topic = format!("{prefix}/{source}/{STOP_TOPIC}");
            let sub = self
                .communication
                .subscribe(&topic)
                .await
                .wrap_err_with(|| format!("failed to subscribe on {topic}"))?;
            stop_messages.push(sub.into_future());
        }
        let finished = Box::pin(stop_messages.all(|_| async { true }));

        Ok(streams.merge().take_until(finished))
    }

    pub async fn send_output(&self, output_id: &DataId, data: &[u8]) -> eyre::Result<()> {
        if !self.operator_config.outputs.contains(output_id) {
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

struct BoxError(Box<dyn std::error::Error + Send + Sync + 'static>);

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
        let operator_config = config::NodeRunConfig {
            inputs: Default::default(),
            outputs: Default::default(),
        };
        let communication_config = config::CommunicationConfig {
            zenoh_config: Default::default(),
            zenoh_prefix: format!("/{}", uuid::Uuid::new_v4()),
        };

        run(async {
            let operator = DoraNode::init(id, operator_config, communication_config)
                .await
                .unwrap();
            let mut inputs = operator.inputs().await.unwrap();
            assert!(inputs.next().await.is_none());
        });
    }
}
*/
