#![warn(unsafe_op_in_unsafe_fn)]

use dora_core::{
    config::{DataId, OperatorId},
    daemon_messages::{NodeConfig, RuntimeConfig},
    descriptor::OperatorConfig,
};
use dora_node_api::DoraNode;
use eyre::{bail, Context, Result};
use futures::{Stream, StreamExt};
use futures_concurrency::stream::Merge;
use operator::{run_operator, OperatorEvent, StopReason};

#[cfg(feature = "tracing")]
use dora_tracing::set_up_tracing;
use std::{
    collections::{BTreeMap, BTreeSet, HashMap},
    mem,
};
use tokio::{
    runtime::Builder,
    sync::{mpsc, oneshot},
};
use tokio_stream::wrappers::ReceiverStream;
mod operator;

pub fn main() -> eyre::Result<()> {
    #[cfg(feature = "tracing")]
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

    let tokio_runtime = Builder::new_current_thread()
        .enable_all()
        .build()
        .wrap_err("Could not build a tokio runtime.")?;

    let mut operator_channels = HashMap::new();
    let queue_sizes = queue_sizes(&operator_definition.config);
    let (operator_channel, incoming_events) =
        operator::channel::channel(tokio_runtime.handle(), queue_sizes);
    operator_channels.insert(operator_definition.id.clone(), operator_channel);

    tracing::info!("spawning main task");
    let operator_config = [(
        operator_definition.id.clone(),
        operator_definition.config.clone(),
    )]
    .into_iter()
    .collect();
    let (init_done_tx, init_done) = oneshot::channel();
    let main_task = std::thread::spawn(move || -> Result<()> {
        tokio_runtime.block_on(run(
            operator_config,
            config,
            operator_events,
            operator_channels,
            init_done,
        ))
    });

    let operator_id = operator_definition.id.clone();
    run_operator(
        &node_id,
        operator_definition,
        incoming_events,
        operator_events_tx,
        init_done_tx,
    )
    .wrap_err_with(|| format!("failed to run operator {operator_id}"))?;

    match main_task.join() {
        Ok(result) => result.wrap_err("main task failed")?,
        Err(panic) => std::panic::resume_unwind(panic),
    }

    Ok(())
}

fn queue_sizes(config: &OperatorConfig) -> std::collections::BTreeMap<DataId, usize> {
    let mut sizes = BTreeMap::new();
    for (input_id, input) in &config.inputs {
        let queue_size = input.queue_size.unwrap_or(10);
        sizes.insert(input_id.clone(), queue_size);
    }
    sizes
}

#[tracing::instrument(skip(operator_events, operator_channels), fields(node.id))]
async fn run(
    operators: HashMap<OperatorId, OperatorConfig>,
    config: NodeConfig,
    operator_events: impl Stream<Item = Event> + Unpin,
    mut operator_channels: HashMap<OperatorId, flume::Sender<operator::IncomingEvent>>,
    init_done: oneshot::Receiver<()>,
) -> eyre::Result<()> {
    #[cfg(feature = "metrics")]
    let _started = {
        use dora_metrics::init_meter;
        use opentelemetry::global;
        use opentelemetry_system_metrics::init_process_observer;

        let _started = init_meter();
        let meter = global::meter(Box::leak(node.id().to_string().into_boxed_str()));
        init_process_observer(meter);
        _started
    };

    init_done
        .await
        .wrap_err("the `init_done` channel was closed unexpectedly")?;
    tracing::info!("All operators are ready, starting runtime");

    let (mut node, daemon_events) = DoraNode::init(config)?;
    let daemon_events = Box::pin(futures::stream::unfold(daemon_events, |mut stream| async {
        let event = stream.recv_async().await.map(|event| match event {
            dora_node_api::Event::Stop => Event::Stop,
            dora_node_api::Event::Input { id, metadata, data } => Event::Input {
                id,
                metadata,
                data: data.map(|data| data.to_owned()),
            },
            dora_node_api::Event::InputClosed { id } => Event::InputClosed(id),
            dora_node_api::Event::Error(err) => Event::Error(err),
            _ => todo!(),
        });
        event.map(|event| (event, stream))
    }));
    let mut events = (operator_events, daemon_events).merge();

    let mut open_operator_inputs: HashMap<_, BTreeSet<_>> = operators
        .iter()
        .map(|(id, config)| (id, config.inputs.keys().collect()))
        .collect();

    while let Some(event) = events.next().await {
        match event {
            Event::Operator {
                id: operator_id,
                event,
            } => {
                match event {
                    OperatorEvent::Error(err) => {
                        bail!(err.wrap_err(format!(
                            "operator {}/{operator_id} raised an error",
                            node.id()
                        )))
                    }
                    OperatorEvent::Panic(payload) => {
                        bail!("operator {operator_id} panicked: {payload:?}");
                    }
                    OperatorEvent::Finished { reason } => {
                        if let StopReason::ExplicitStopAll = reason {
                            // let hlc = dora_core::message::uhlc::HLC::default();
                            // let metadata = dora_core::message::Metadata::new(hlc.new_timestamp());
                            // let data = metadata
                            // .serialize()
                            // .wrap_err("failed to serialize stop message")?;
                            todo!("instruct dora-daemon/dora-coordinator to stop other nodes");
                            // manual_stop_publisher
                            //     .publish(&data)
                            //     .map_err(|err| eyre::eyre!(err))
                            //     .wrap_err("failed to send stop message")?;
                            // break;
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
                        .await
                        .wrap_err("failed to wait for close_outputs task")?;
                        result.wrap_err("failed to close outputs of finished operator")?;

                        operator_channels.remove(&operator_id);

                        if operator_channels.is_empty() {
                            break;
                        }
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
                        .await
                        .wrap_err("failed to wait for send_output task")?;
                        result.wrap_err("failed to send node output")?;
                    }
                }
            }
            Event::Stop => {
                // forward stop event to all operators and close the event channels
                for (_, channel) in operator_channels.drain() {
                    let _ = channel.send_async(operator::IncomingEvent::Stop).await;
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

                if let Err(err) = operator_channel
                    .send_async(operator::IncomingEvent::Input {
                        input_id: input_id.clone(),
                        metadata,
                        data,
                    })
                    .await
                    .wrap_err_with(|| {
                        format!("failed to send input `{input_id}` to operator `{operator_id}`")
                    })
                {
                    tracing::warn!("{err}");
                }
            }
            Event::InputClosed(id) => {
                let Some((operator_id, input_id)) = id.as_str().split_once('/') else {
                    tracing::warn!("received InputClosed event for non-operator input {id}");
                    continue;
                };
                let operator_id = OperatorId::from(operator_id.to_owned());
                let input_id = DataId::from(input_id.to_owned());

                let Some(operator_channel) = operator_channels.get(&operator_id) else {
                    tracing::warn!("received input {id} for unknown operator");
                    continue;
                };
                if let Err(err) = operator_channel
                    .send_async(operator::IncomingEvent::InputClosed {
                        input_id: input_id.clone(),
                    })
                    .await
                    .wrap_err_with(|| {
                        format!(
                            "failed to send InputClosed({input_id}) to operator `{operator_id}`"
                        )
                    })
                {
                    tracing::warn!("{err}");
                }

                if let Some(open_inputs) = open_operator_inputs.get_mut(&operator_id) {
                    open_inputs.remove(&input_id);
                    if open_inputs.is_empty() {
                        // all inputs of the node were closed -> close its event channel
                        tracing::info!("all inputs of operator {}/{operator_id} were closed -> closing event channel", node.id());
                        open_operator_inputs.remove(&operator_id);
                        operator_channels.remove(&operator_id);
                    }
                }
            }
            Event::Error(err) => eyre::bail!("received error event: {err}"),
        }
    }

    mem::drop(events);

    Ok(())
}

fn operator_output_id(operator_id: &OperatorId, output_id: &DataId) -> DataId {
    DataId::from(format!("{operator_id}/{output_id}"))
}

#[derive(Debug)]
enum Event {
    Operator {
        id: OperatorId,
        event: OperatorEvent,
    },
    Stop,
    Input {
        id: dora_core::config::DataId,
        metadata: dora_core::message::Metadata<'static>,
        data: Option<Vec<u8>>,
    },
    InputClosed(dora_core::config::DataId),
    Error(String),
}
