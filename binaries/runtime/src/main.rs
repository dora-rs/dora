#![warn(unsafe_op_in_unsafe_fn)]

use dora_core::descriptor::OperatorDefinition;
use dora_node_api::{
    self,
    communication::{self, CommunicationLayer, Publisher, STOP_TOPIC},
    config::{CommunicationConfig, DataId, NodeId, OperatorId},
};
use eyre::{bail, Context};
use futures::{Stream, StreamExt};
use operator::{spawn_operator, OperatorEvent};
use std::{
    collections::{BTreeSet, HashMap},
    mem,
};
use tokio::sync::mpsc;
use tokio_stream::{wrappers::ReceiverStream, StreamMap};

mod operator;

fn main() -> eyre::Result<()> {
    set_up_logger()?;

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
    let operators: Vec<OperatorDefinition> = {
        let raw =
            std::env::var("DORA_OPERATORS").wrap_err("env variable DORA_OPERATORS must be set")?;
        serde_yaml::from_str(&raw).context("failed to deserialize operator config")?
    };

    let mut communication: Box<dyn CommunicationLayer> =
        communication::init(&communication_config)?;

    let mut operator_events = StreamMap::new();
    let mut operator_stop_publishers = HashMap::new();
    for operator_config in &operators {
        let (events_tx, events) = mpsc::channel(1);
        spawn_operator(
            &node_id,
            operator_config.clone(),
            events_tx.clone(),
            communication.as_mut(),
        )
        .wrap_err_with(|| format!("failed to init operator {}", operator_config.id))?;
        operator_events.insert(operator_config.id.clone(), ReceiverStream::new(events));

        let stop_publisher = publisher(
            &node_id,
            operator_config.id.clone(),
            STOP_TOPIC.to_owned().into(),
            communication.as_mut(),
        )
        .with_context(|| {
            format!(
                "failed to create stop publisher for operator {}",
                operator_config.id
            )
        })?;
        operator_stop_publishers.insert(operator_config.id.clone(), stop_publisher);
    }

    let operator_events = operator_events.map(|(id, event)| Event::Operator { id, event });

    tokio::runtime::Runtime::new()?.block_on(run(
        node_id,
        operator_events,
        operator_stop_publishers,
    ))
}

async fn run(
    node_id: NodeId,
    mut events: impl Stream<Item = Event> + Unpin,
    mut operator_stop_publishers: HashMap<OperatorId, Box<dyn Publisher>>,
) -> eyre::Result<()> {
    let mut stopped_operators = BTreeSet::new();

    while let Some(event) = events.next().await {
        match event {
            Event::Operator { id, event } => {
                match event {
                    OperatorEvent::Error(err) => {
                        bail!(err.wrap_err(format!("operator {id} failed")))
                    }
                    OperatorEvent::Panic(payload) => std::panic::resume_unwind(payload),
                    OperatorEvent::Finished => {
                        if let Some(stop_publisher) = operator_stop_publishers.remove(&id) {
                            println!("operator {node_id}/{id} finished");
                            stopped_operators.insert(id.clone());
                            // send stopped message
                            tokio::task::spawn_blocking(move || stop_publisher.publish(&[]))
                                .await
                                .wrap_err("failed to join stop publish task")?
                                .map_err(|err| eyre::eyre!(err))
                                .with_context(|| {
                                    format!(
                                        "failed to send stop message for operator `{node_id}/{id}`"
                                    )
                                })?;
                            if operator_stop_publishers.is_empty() {
                                break;
                            }
                        } else {
                            log::warn!("no stop publisher for {id}");
                        }
                    }
                }
            }
        }
    }

    mem::drop(events);

    Ok(())
}

fn publisher(
    self_id: &NodeId,
    operator_id: OperatorId,
    output_id: DataId,
    communication: &mut dyn CommunicationLayer,
) -> eyre::Result<Box<dyn Publisher>> {
    let topic = format!("{self_id}/{operator_id}/{output_id}");
    communication
        .publisher(&topic)
        .map_err(|err| eyre::eyre!(err))
        .wrap_err_with(|| format!("failed to create publisher for output {output_id}"))
}

enum Event {
    Operator {
        id: OperatorId,
        event: OperatorEvent,
    },
}

fn set_up_logger() -> Result<(), fern::InitError> {
    fern::Dispatch::new()
        .format(|out, message, record| {
            out.finish(format_args!(
                "    [{}][{}] {}",
                record.target(),
                record.level(),
                message
            ))
        })
        .level(log::LevelFilter::Debug)
        .level_for("zenoh", log::LevelFilter::Warn)
        .level_for("zenoh_transport", log::LevelFilter::Warn)
        .level_for("zenoh_link_tcp", log::LevelFilter::Warn)
        .chain(std::io::stdout())
        .chain(fern::log_file("runtime.log")?)
        .apply()?;
    Ok(())
}
