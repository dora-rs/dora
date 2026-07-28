#![warn(unsafe_op_in_unsafe_fn)]

use dora_core::{
    config::{DataId, OperatorId},
    descriptor::OperatorConfig,
};
use dora_message::daemon_to_node::{NodeConfig, RuntimeConfig};
use dora_node_api::{DoraNode, Event};
use dora_tracing::TracingBuilder;
use eyre::{Context, Result, bail};
use futures::{Stream, StreamExt};
use futures_concurrency::stream::Merge;
use std::{
    collections::{BTreeMap, BTreeSet, HashMap},
    mem,
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};
use tokio::{
    runtime::Builder,
    sync::{mpsc, oneshot},
};
use tokio_stream::wrappers::ReceiverStream;

mod channel;
mod operator;

pub use operator::{OperatorEvent, OperatorRunner, RunnerGuard, StopReason};

/// Entry point for a runtime process.
///
/// Reads the `DORA_RUNTIME_CONFIG` env var, builds the tokio runtime, runs the
/// language-neutral event loop on a spawned thread, and calls
/// [`OperatorRunner::run_operator`] on the main thread. Each per-language
/// backend calls this with its own runner.
pub fn main(runner: impl OperatorRunner) -> eyre::Result<()> {
    let config: RuntimeConfig = {
        let raw = std::env::var("DORA_RUNTIME_CONFIG")
            .wrap_err("env variable DORA_RUNTIME_CONFIG must be set")?;
        serde_yaml::from_str(&raw).context("failed to deserialize runtime config")?
    };
    let RuntimeConfig {
        node: config,
        operators,
    } = config;
    let node_id = config.node_id.clone();
    #[cfg(feature = "tracing")]
    {
        TracingBuilder::new(node_id.as_ref())
            .with_stdout("warn", false)
            .build()
            .wrap_err("failed to set up tracing subscriber")?;
    }

    let dataflow_descriptor = serde_yaml::from_value(config.dataflow_descriptor.clone())
        .context("failed to parse dataflow descriptor")?;

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
    let operator_events = ReceiverStream::new(events).map(move |event| RuntimeEvent::Operator {
        id: operator_id.clone(),
        event,
    });

    // Use multi-thread scheduler (with 1 worker) because Zenoh requires it
    // for distributed cross-daemon communication. current_thread panics when
    // Zenoh tries to spawn background tasks during session init.
    let tokio_runtime = Builder::new_multi_thread()
        .worker_threads(1)
        .enable_all()
        .build()
        .wrap_err("Could not build a tokio runtime.")?;

    let mut operator_channels = HashMap::new();
    let queue_sizes = queue_sizes(&operator_definition.config);
    let (operator_channel, incoming_events) = channel::channel(tokio_runtime.handle(), queue_sizes);
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
    // Hold the backend's guard until *after* the main event loop has joined
    // below. The shared-library backend returns the loaded `.so` here: the
    // operator's outputs are Arrow arrays whose FFI `release` callbacks live
    // inside that library, and the loop may still hold in-flight arrays.
    // Unloading it earlier dangles those callbacks and SIGSEGVs when the arrays
    // are freed. See [`RunnerGuard`].
    let _operator_guard = runner
        .run_operator(
            &node_id,
            operator_definition,
            incoming_events,
            operator_events_tx,
            init_done_tx,
            &dataflow_descriptor,
        )
        .wrap_err_with(|| format!("failed to run operator {operator_id}"))?;

    match main_task.join() {
        Ok(result) => result.wrap_err("main task failed")?,
        Err(panic) => std::panic::resume_unwind(panic),
    }

    // `_operator_guard` drops here (unloading the `.so` for the shared-library
    // backend), after the main loop has joined and released every Arrow array
    // the operator exported.
    Ok(())
}

fn queue_sizes(
    config: &OperatorConfig,
) -> std::collections::BTreeMap<DataId, (usize, dora_message::config::QueuePolicy)> {
    let mut sizes = BTreeMap::new();
    for (input_id, input) in &config.inputs {
        let queue_size = input
            .queue_size
            .unwrap_or(dora_message::config::DEFAULT_QUEUE_SIZE);
        let policy = input.queue_policy.unwrap_or_default();
        sizes.insert(input_id.clone(), (queue_size, policy));
    }
    sizes
}

#[tracing::instrument(skip(operator_events, operator_channels), level = "trace")]
async fn run(
    operators: HashMap<OperatorId, OperatorConfig>,
    config: NodeConfig,
    operator_events: impl Stream<Item = RuntimeEvent> + Unpin,
    mut operator_channels: HashMap<OperatorId, flume::Sender<Event>>,
    init_done: oneshot::Receiver<Result<()>>,
) -> eyre::Result<()> {
    // Start the OTLP metrics exporter only when an endpoint is configured, and
    // spawn it as a background task. `run_metrics_monitor` is an `async fn`, so
    // its returned future does nothing until polled; previously the future was
    // bound to a `_meter_provider` local and dropped without ever being awaited
    // or spawned, so the `metrics` feature silently exported nothing. The future
    // also never resolves (the process observer runs for the node's lifetime),
    // so it must be spawned rather than awaited inline. Mirrors the gating and
    // spawning used by the node API (`apis/rust/node/src/node/mod.rs`).
    #[cfg(feature = "metrics")]
    if std::env::var("DORA_OTLP_ENDPOINT").is_ok() {
        use dora_metrics::run_metrics_monitor;

        let meter_id = config.node_id.to_string();
        tokio::spawn(async move {
            if let Err(e) = run_metrics_monitor(meter_id)
                .await
                .wrap_err("metrics monitor exited unexpectedly")
            {
                tracing::warn!("metrics monitor failed: {e:#}");
            }
        });
    }
    init_done
        .await
        .wrap_err("the `init_done` channel was closed unexpectedly")?
        .wrap_err("failed to init an operator")?;
    tracing::info!("All operators are ready, starting runtime");

    let (mut node, mut daemon_events) = DoraNode::init(config)?;
    let (daemon_events_tx, daemon_event_stream) = flume::bounded(1);
    tokio::task::spawn_blocking(move || {
        while let Some(event) = daemon_events.recv() {
            if daemon_events_tx.send(RuntimeEvent::Event(event)).is_err() {
                break;
            }
        }
    });
    let mut events = (operator_events, daemon_event_stream.into_stream()).merge();

    let mut open_operator_inputs: HashMap<_, BTreeSet<_>> = operators
        .iter()
        .map(|(id, config)| (id, config.inputs.keys().collect()))
        .collect();

    // Diagnostic watchdog (dora-rs/dora#2742): warn when the main loop stops
    // making progress *while handling* an event, naming the stuck event. On
    // Windows a wedged operator can't be soft-killed (`CTRL_BREAK_EVENT` cannot
    // interrupt native code the way Unix `SIGTERM` does), so the only symptom is
    // a silent grace-period force-kill with no clue where it parked.
    let activity = Arc::new(Mutex::new(LoopActivity::Idle));
    {
        let activity = activity.clone();
        let node_id = node.id().to_string();
        tokio::spawn(async move {
            let mut ticker = tokio::time::interval(Duration::from_secs(2));
            ticker.tick().await; // first tick fires immediately
            loop {
                ticker.tick().await;
                let stalled = match &*lock(&activity) {
                    LoopActivity::Handling { since, what }
                        if since.elapsed() > Duration::from_secs(3) =>
                    {
                        Some((since.elapsed(), *what))
                    }
                    _ => None,
                };
                if let Some((elapsed, what)) = stalled {
                    tracing::warn!(
                        "runtime `{node_id}` main loop stalled for {:.0}s while handling {what} \
                         (dora-rs/dora#2742 diagnostic)",
                        elapsed.as_secs_f32()
                    );
                }
            }
        });
    }

    loop {
        *lock(&activity) = LoopActivity::Idle;
        let Some(event) = events.next().await else {
            break;
        };
        *lock(&activity) = LoopActivity::Handling {
            since: Instant::now(),
            what: describe_runtime_event(&event),
        };
        match event {
            RuntimeEvent::Operator {
                id: operator_id,
                event,
            } => match event {
                OperatorEvent::Error(err) => {
                    bail!(err.wrap_err(format!(
                        "operator {}/{operator_id} raised an error",
                        node.id()
                    )))
                }
                OperatorEvent::Panic(payload) => {
                    let message = payload
                        .downcast_ref::<&str>()
                        .map(|s| s.to_string())
                        .or_else(|| payload.downcast_ref::<String>().cloned())
                        .unwrap_or_else(|| format!("{payload:?}"));
                    bail!("operator {operator_id} panicked: {message}");
                }
                OperatorEvent::Finished { reason } => {
                    if let StopReason::ExplicitStopAll = reason {
                        bail!(
                            "operator {operator_id} requested StopAll, which is not yet implemented"
                        );
                    }

                    let Some(config) = operators.get(&operator_id) else {
                        tracing::warn!(
                            "received Finished event for unknown operator `{operator_id}`"
                        );
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
                    parameters,
                    arrow_array,
                } => {
                    let output_id = operator_output_id(&operator_id, &output_id);
                    let result;
                    (node, result) = tokio::task::spawn_blocking(move || {
                        let array = dora_node_api::arrow::array::make_array(arrow_array);
                        let result = node.send_output(output_id, parameters, array);
                        (node, result)
                    })
                    .await
                    .wrap_err("failed to wait for send_output task")?;
                    result.wrap_err("failed to send node output")?;
                }
            },
            RuntimeEvent::Event(Event::Stop(cause)) => {
                // Diagnostic (dora-rs/dora#2742): trace Stop delivery so a
                // Windows nightly shows whether the runtime received Stop and
                // whether each per-operator forward *completed*. A logged
                // "received Stop" with no matching "forwarded Stop" means the
                // forward blocked on a full operator channel — i.e. the operator
                // is parked in its own `on_event` and never draining.
                // `warn!` (not `info!`) so it survives the runtime's default
                // stdout filter (`with_stdout("warn", …)`) and reaches the
                // nightly log.
                tracing::warn!(
                    "runtime received Stop; forwarding to {} operator(s) (dora-rs/dora#2742 diagnostic)",
                    operator_channels.len()
                );
                // forward stop event to all operators and close the event channels
                for (id, channel) in operator_channels.drain() {
                    let _ = channel.send_async(Event::Stop(cause.clone())).await;
                    tracing::warn!(
                        "forwarded Stop to operator `{id}` (dora-rs/dora#2742 diagnostic)"
                    );
                }
            }
            RuntimeEvent::Event(Event::Reload {
                operator_id: Some(operator_id),
            }) => {
                let Some(operator_channel) = operator_channels.get(&operator_id) else {
                    tracing::warn!("received Reload event for unknown operator `{operator_id}`");
                    continue;
                };
                let _ = operator_channel
                    .send_async(Event::Reload {
                        operator_id: Some(operator_id),
                    })
                    .await;
            }
            RuntimeEvent::Event(Event::Reload { operator_id: None }) => {
                tracing::warn!("Reloading runtime nodes is not supported");
            }
            RuntimeEvent::Event(Event::Input { id, metadata, data }) => {
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
                    .send_async(Event::Input {
                        id: input_id,
                        metadata,
                        data,
                    })
                    .await
                    .wrap_err_with(|| {
                        // `id` is the full `operator/input` DataId; use it (and the
                        // still-owned `operator_id`) here so `input_id` can be moved
                        // into the event above without a per-message clone.
                        format!("failed to send input `{id}` to operator `{operator_id}`")
                    })
                {
                    tracing::warn!("{err}");
                }
            }
            RuntimeEvent::Event(Event::InputClosed { id }) => {
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
                    .send_async(Event::InputClosed {
                        id: input_id.clone(),
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
                        tracing::trace!(
                            "all inputs of operator {}/{operator_id} were closed -> closing event channel",
                            node.id()
                        );
                        open_operator_inputs.remove(&operator_id);
                        operator_channels.remove(&operator_id);
                    }
                }
            }
            RuntimeEvent::Event(Event::Error(err)) => eyre::bail!("received error event: {err}"),
            RuntimeEvent::Event(other) => {
                tracing::warn!("received unknown event `{other:?}`");
            }
        }
    }

    mem::drop(events);

    Ok(())
}

fn operator_output_id(operator_id: &OperatorId, output_id: &DataId) -> DataId {
    DataId::from(format!("{operator_id}/{output_id}"))
}

#[derive(Debug)]
enum RuntimeEvent {
    Operator {
        id: OperatorId,
        event: OperatorEvent,
    },
    Event(Event),
}

/// What the runtime's main loop is currently doing, for the stall watchdog.
///
/// Diagnostic for dora-rs/dora#2742: on Windows a wedged operator cannot be
/// interrupted by the daemon's soft-kill (`CTRL_BREAK_EVENT`, unlike Unix
/// `SIGTERM`), so the node runs to the force-kill and the failure shows up only
/// as a grace-period kill with no clue where it parked. The watchdog names the
/// event whose handling has stopped making progress.
enum LoopActivity {
    Idle,
    Handling { since: Instant, what: &'static str },
}

/// Which *kind* of runtime event the main loop is handling, for the stall
/// watchdog. Returns a `&'static str` (no per-event allocation on the hot path);
/// the kind alone distinguishes the two wedge sites that matter — a stalled
/// `an operator output` is a blocked daemon send, a stalled `an operator input`
/// is a blocked forward to a parked operator.
fn describe_runtime_event(event: &RuntimeEvent) -> &'static str {
    match event {
        RuntimeEvent::Operator { event, .. } => match event {
            OperatorEvent::Output { .. } => "an operator output",
            _ => "an operator lifecycle event",
        },
        RuntimeEvent::Event(event) => match event {
            Event::Input { .. } => "an operator input",
            Event::InputClosed { .. } => "an input-closed event",
            Event::Stop(_) => "a stop event",
            Event::Reload { .. } => "a reload event",
            _ => "an event",
        },
    }
}

/// Lock a mutex, recovering the guard even if a previous holder panicked. The
/// watchdog state is pure diagnostics, so a poisoned lock must not take the
/// runtime down with it.
fn lock<T>(mutex: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    mutex
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
}
