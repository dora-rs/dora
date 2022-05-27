#![warn(unsafe_op_in_unsafe_fn)]

use dora_common::{descriptor::OperatorConfig, BoxError};
use dora_node_api::{
    self,
    communication::CommunicationLayer,
    config::{CommunicationConfig, DataId, InputMapping, NodeId, OperatorId},
    STOP_TOPIC,
};
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
