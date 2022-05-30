#![warn(unsafe_op_in_unsafe_fn)]

use dora_common::{descriptor::OperatorConfig, BoxError};
use dora_node_api::{
    self,
    communication::CommunicationLayer,
    config::{CommunicationConfig, DataId, InputMapping, NodeId, OperatorId},
    STOP_TOPIC,
};
use eyre::{bail, eyre, Context};
use futures::{
    stream::{self, FuturesUnordered},
    FutureExt, StreamExt,
};
use futures_concurrency::Merge;
use operator::{Operator, OperatorEvent};
use std::{collections::BTreeMap, mem};
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

    let inputs = subscribe(&operators, communication.as_ref(), &communication_config)
        .await
        .context("failed to subscribe")?;

    let input_events = inputs.map(Event::External);
    let operator_events = operator_events.map(|(id, event)| Event::Operator { id, event });
    let mut events = (input_events, operator_events).merge();

    while let Some(event) = events.next().await {
        match event {
            Event::External(event) => match event {
                SubscribeEvent::Input(input) => {
                    let operator =
                        operator_map
                            .get_mut(&input.target_operator)
                            .ok_or_else(|| {
                                eyre!(
                                    "received input for unexpected operator `{}`",
                                    input.target_operator
                                )
                            })?;

                    operator
                        .handle_input(input.id.clone(), input.data)
                        .wrap_err_with(|| {
                            format!(
                                "operator {} failed to handle input {}",
                                input.target_operator, input.id
                            )
                        })?;
                }
                SubscribeEvent::InputsStopped { target_operator } => {
                    // --------------------------------------------------------
                    // TODO FIXME: For some reason, these zenoh publish calls
                    // (and also subsequent ones) are not visible to other
                    // nodes. This includes the stop command, so the input
                    // streams of dependent nodes are not closed properly.
                    // --------------------------------------------------------

                    communication
                        .publish("/HHH", &[])
                        .await
                        .wrap_err("failed to send on /HHH")?;
                    if operator_map.remove(&target_operator).is_some() {
                        println!("operator {node_id}/{target_operator} finished");
                        // send stopped message
                        publish(
                            &node_id,
                            target_operator.clone(),
                            STOP_TOPIC.to_owned().into(),
                            &[],
                            communication.as_ref(),
                            &communication_config,
                        )
                        .await.with_context(|| {
                            format!("failed to send stop message for operator `{node_id}/{target_operator}`")
                        })?;
                    }

                    if operator_map.is_empty() {
                        break;
                    }
                }
            },
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

    mem::drop(events);

    communication.close().await?;

    Ok(())
}

async fn subscribe<'a>(
    operators: &'a [OperatorConfig],
    communication: &'a dyn CommunicationLayer,
    communication_config: &CommunicationConfig,
) -> eyre::Result<impl futures::Stream<Item = SubscribeEvent> + 'a> {
    let mut streams = Vec::new();

    for operator in operators {
        let events = subscribe_operator(operator, communication, communication_config).await?;
        streams.push(events);
    }

    Ok(streams.merge())
}

async fn subscribe_operator<'a>(
    operator: &'a OperatorConfig,
    communication: &'a dyn CommunicationLayer,
    communication_config: &CommunicationConfig,
) -> Result<impl futures::Stream<Item = SubscribeEvent> + 'a, eyre::Error> {
    let prefix = &communication_config.zenoh_prefix;

    let stop_messages = FuturesUnordered::new();
    for input in operator.inputs.values() {
        let InputMapping {
            source, operator, ..
        } = input;
        let topic = match operator {
            Some(operator) => format!("{prefix}/{source}/{operator}/{STOP_TOPIC}"),
            None => format!("{prefix}/{source}/{STOP_TOPIC}"),
        };
        let sub = communication
            .subscribe(&topic)
            .await
            .wrap_err_with(|| format!("failed to subscribe on {topic}"))?;
        stop_messages.push(sub.into_future());
    }
    let finished = Box::pin(stop_messages.all(|_| async { true }).shared());

    let mut streams = Vec::new();
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
        let sub = communication
            .subscribe(&topic)
            .await
            .wrap_err_with(|| format!("failed to subscribe on {topic}"))?;
        let stream = sub
            .map(|data| OperatorInput {
                target_operator: operator.id.clone(),
                id: input.clone(),
                data,
            })
            .map(SubscribeEvent::Input)
            .take_until(finished.clone())
            .chain(stream::once(async {
                SubscribeEvent::InputsStopped {
                    target_operator: operator.id.clone(),
                }
            }));
        streams.push(stream);
    }

    Ok(streams.merge())
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
    External(SubscribeEvent),
    Operator {
        id: OperatorId,
        event: OperatorEvent,
    },
}

enum SubscribeEvent {
    /// New input for an operator
    Input(OperatorInput),
    /// All input streams for an operator are finished.
    InputsStopped {
        /// The operator whose inputs are all finished.
        target_operator: OperatorId,
    },
}

struct OperatorInput {
    pub target_operator: OperatorId,
    pub id: DataId,
    pub data: Vec<u8>,
}
