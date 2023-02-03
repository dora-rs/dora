#![warn(unsafe_op_in_unsafe_fn)]

use dora_core::{
    config::{DataId, OperatorId},
    daemon_messages::RuntimeConfig,
    descriptor::OperatorConfig,
};
use dora_node_api::DoraNode;
use eyre::{bail, Context, Result};
use futures::{Stream, StreamExt};
use futures_concurrency::Merge;
use operator::{run_operator, OperatorEvent, StopReason};

use std::{collections::HashMap, mem};
use tokio::{runtime::Builder, sync::mpsc};
use tokio_stream::wrappers::ReceiverStream;

mod operator;

pub fn main() -> eyre::Result<()> {
    set_up_tracing().context("failed to set up tracing subscriber")?;

    let config: RuntimeConfig = {
        let raw = std::env::var("DORA_RUNTIME_CONFIG")
            .wrap_err("env variable DORA_RUNTIME_CONFIG must be set")?;
        serde_yaml::from_str(&raw).context("failed to deserialize operator config")?
    };
    let RuntimeConfig {
        node: config,
        operators,
    } = config;
    let node_id = config.node_id.clone();
    let (node, daemon_events) = DoraNode::init(config)?;

    let operator_definition = if operators.is_empty() {
        bail!("no operators");
    } else if operators.len() > 1 {
        bail!("multiple operators are not supported");
    } else {
        let mut ops = operators;
        ops.remove(0)
    };

    let (operator_events_tx, events) = mpsc::channel(1);
    let operator_id = operator_definition.id.clone();
    let operator_events = ReceiverStream::new(events).map(move |event| Event::Operator {
        id: operator_id.clone(),
        event,
    });
    let daemon_events = futures::stream::unfold(daemon_events, |mut stream| async {
        let event = stream.recv_async().await.map(|event| match event {
            dora_node_api::daemon::Event::Stop => Event::Stop,
            dora_node_api::daemon::Event::Input { id, metadata, data } => Event::Input {
                id,
                metadata,
                data: data.map(|data| data.to_owned()),
            },
            dora_node_api::daemon::Event::InputClosed { id } => Event::InputClosed(id),
            dora_node_api::daemon::Event::Error(err) => Event::Error(err),
            _ => todo!(),
        });
        event.map(|event| (event, stream))
    });
    let events = (operator_events, daemon_events).merge();
    let tokio_runtime = Builder::new_current_thread()
        .enable_all()
        .build()
        .wrap_err("Could not build a tokio runtime.")?;

    let mut operator_channels = HashMap::new();
    let (operator_channel, incoming_events) = mpsc::channel(10);
    operator_channels.insert(operator_definition.id.clone(), operator_channel);

    tracing::info!("spawning main task");
    let operator_config = [(
        operator_definition.id.clone(),
        operator_definition.config.clone(),
    )]
    .into_iter()
    .collect();
    let main_task = std::thread::spawn(move || -> Result<()> {
        tokio_runtime.block_on(run(node, operator_config, events, operator_channels))
    });

    let operator_id = operator_definition.id.clone();
    run_operator(
        &node_id,
        operator_definition,
        incoming_events,
        operator_events_tx,
    )
    .wrap_err_with(|| format!("failed to run operator {operator_id}"))?;

    main_task
        .join()
        .map_err(|err| eyre::eyre!("Stop thread failed with err: {err:#?}"))?
        .wrap_err("Stop loop thread failed unexpectedly.")?;

    Ok(())
}

#[tracing::instrument(skip(node, events, operator_channels), fields(node.id))]
async fn run(
    mut node: DoraNode,
    operators: HashMap<OperatorId, OperatorConfig>,
    mut events: impl Stream<Item = Event> + Unpin,
    mut operator_channels: HashMap<OperatorId, mpsc::Sender<operator::IncomingEvent>>,
) -> eyre::Result<()> {
    #[cfg(feature = "metrics")]
    let _started = {
        use dora_metrics::init_meter;
        use opentelemetry::global;
        use opentelemetry_system_metrics::init_process_observer;

        let _started = init_meter();
        let meter = global::meter(Box::leak(node_id.to_string().into_boxed_str()));
        init_process_observer(meter);
        _started
    };

    // let mut stopped_operators = BTreeSet::new();

    while let Some(event) = events.next().await {
        match event {
            Event::Operator {
                id: operator_id,
                event,
            } => {
                match event {
                    OperatorEvent::Error(err) => {
                        bail!(err.wrap_err(format!("operator {operator_id} failed")))
                    }
                    OperatorEvent::Panic(payload) => std::panic::resume_unwind(payload),
                    OperatorEvent::Finished { reason } => {
                        if let StopReason::ExplicitStopAll = reason {
                            let hlc = dora_message::uhlc::HLC::default();
                            let metadata = dora_message::Metadata::new(hlc.new_timestamp());
                            let data = metadata
                                .serialize()
                                .wrap_err("failed to serialize stop message")?;
                            todo!("instruct dora-daemon/dora-coordinator to stop other nodes");
                            // manual_stop_publisher
                            //     .publish(&data)
                            //     .map_err(|err| eyre::eyre!(err))
                            //     .wrap_err("failed to send stop message")?;
                            break;
                        }

                        let Some(config) = operators.get(&operator_id) else {
                            tracing::warn!("received Finished event for unknown operator `{operator_id}`");
                            continue;
                        };
                        let outputs = config
                            .outputs
                            .iter()
                            .map(|output_id| operator_output_id(&operator_id, output_id))
                            .collect();
                        let result;
                        (node, result) = tokio::task::spawn_blocking(move || {
                            let result = node.close_outputs(outputs);
                            (node, result)
                        })
                        .await?;
                        result.wrap_err("failed to close outputs of finished operator")?;
                    }
                    OperatorEvent::Output {
                        output_id,
                        metadata,
                        data,
                    } => {
                        let output_id = operator_output_id(&operator_id, &output_id);
                        let result;
                        (node, result) = tokio::task::spawn_blocking(move || {
                            let result = node.send_output(output_id, metadata, data.len(), |buf| {
                                buf.copy_from_slice(&data);
                            });
                            (node, result)
                        })
                        .await?;
                        result.wrap_err("failed to send node output")?;
                    }
                }
            }
            Event::Stop => {
                // forward stop event to all operators and close the event channels
                for (_, channel) in operator_channels.drain() {
                    let _ = channel.send(operator::IncomingEvent::Stop).await;
                }
            }
            Event::Input { id, metadata, data } => {
                let Some((operator_id, input_id)) = id.as_str().split_once('/') else {
                    tracing::warn!("received non-operator input {id}");
                    continue;
                };
                let operator_id = OperatorId::from(operator_id.to_owned());
                let input_id = DataId::from(input_id.to_owned());
                let Some(operator_channel) = operator_channels.get(&operator_id) else {
                    tracing::warn!("received input {id} for unknown operator");
                    continue;
                };

                let _ = operator_channel.send(operator::IncomingEvent::Input {
                    input_id,
                    metadata,
                    data,
                });
            }
            Event::InputClosed(_) => {}
            Event::Error(err) => eyre::bail!("received error event: {err}"),
        }
    }

    mem::drop(events);

    Ok(())
}

fn operator_output_id(operator_id: &OperatorId, output_id: &DataId) -> DataId {
    DataId::from(format!("{operator_id}/{output_id}"))
}

enum Event {
    Operator {
        id: OperatorId,
        event: OperatorEvent,
    },
    Stop,
    Input {
        id: dora_core::config::DataId,
        metadata: dora_message::Metadata<'static>,
        data: Option<Vec<u8>>,
    },
    InputClosed(dora_core::config::DataId),
    Error(String),
}

fn set_up_tracing() -> eyre::Result<()> {
    use tracing_subscriber::prelude::__tracing_subscriber_SubscriberExt;

    let stdout_log = tracing_subscriber::fmt::layer().pretty();
    let subscriber = tracing_subscriber::Registry::default().with(stdout_log);
    tracing::subscriber::set_global_default(subscriber)
        .context("failed to set tracing global subscriber")
}
