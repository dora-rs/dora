#![warn(unsafe_op_in_unsafe_fn)]

use dora_core::{
    config::{DataId, OperatorId},
    descriptor::OperatorConfig,
};
use dora_message::daemon_to_node::{NodeConfig, RuntimeConfig};
use dora_node_api::{DoraNode, Event, StopCause};
use dora_tracing::TracingBuilder;
use eyre::{Context, Result, bail};
use futures::{Stream, StreamExt};
use futures_concurrency::stream::Merge;
use operator::{OperatorEvent, RuntimeHandle, SharedAllocator, StopReason, run_operator};

use std::{
    collections::{BTreeMap, BTreeSet, HashMap, VecDeque},
    mem,
    sync::{Arc, Mutex, OnceLock},
    time::{Duration, Instant},
};
use tokio::{
    runtime::Builder,
    sync::{mpsc, oneshot},
};
use tokio_stream::wrappers::ReceiverStream;
mod operator;

pub fn main() -> eyre::Result<()> {
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
    // Filled by `run` as soon as the node exists; operators encode their outputs
    // through it (see `SharedAllocator`, dora-rs/dora#2742).
    let allocator: SharedAllocator = Arc::new(OnceLock::new());
    let run_allocator = allocator.clone();
    let main_task = std::thread::spawn(move || -> Result<()> {
        tokio_runtime.block_on(run(
            operator_config,
            config,
            operator_events,
            operator_channels,
            init_done,
            run_allocator,
        ))
    });

    let operator_id = operator_definition.id.clone();
    // Keep the operator's shared library mapped until *after* the main event
    // loop has joined below. Since dora-rs/dora#2742 the loop no longer holds
    // Arrow arrays exported by the operator, but values whose vtable lives in
    // the `.so` can still be in flight (an `OperatorEvent::Panic` payload).
    // Unloading it earlier dangles those (see `run_operator` /
    // `shared_lib::run`).
    let _operator_library = run_operator(
        &node_id,
        operator_definition,
        incoming_events,
        RuntimeHandle::new(operator_events_tx, allocator),
        init_done_tx,
        &dataflow_descriptor,
    )
    .wrap_err_with(|| format!("failed to run operator {operator_id}"))?;

    match main_task.join() {
        Ok(result) => result.wrap_err("main task failed")?,
        Err(panic) => std::panic::resume_unwind(panic),
    }

    // `_operator_library` drops (unloads the `.so`) at end of scope here, after
    // the main loop has joined and released everything the operator handed it.
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

#[tracing::instrument(skip(operator_events, operator_channels, allocator), level = "trace")]
async fn run(
    operators: HashMap<OperatorId, OperatorConfig>,
    config: NodeConfig,
    operator_events: impl Stream<Item = RuntimeEvent> + Unpin,
    mut operator_channels: HashMap<OperatorId, flume::Sender<Event>>,
    init_done: oneshot::Receiver<Result<()>>,
    allocator: SharedAllocator,
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
    // Publish the allocator before any input can reach an operator, so an
    // operator's first `send_output` already has somewhere to encode into.
    let _ = allocator.set(node.sample_allocator());
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

    // Events pulled off the stream while a send was in flight (see
    // `await_send_watching_for_stop`); they are handled before the stream is
    // polled again so ordering is preserved.
    let mut pending: VecDeque<RuntimeEvent> = VecDeque::new();

    loop {
        *lock(&activity) = LoopActivity::Idle;
        let next = match pending.pop_front() {
            buffered @ Some(_) => buffered,
            None => events.next().await,
        };
        let Some(event) = next else { break };
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
                    )));
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
                    encoded,
                } => {
                    let output_id = operator_output_id(&operator_id, &output_id);
                    let mut send = tokio::task::spawn_blocking(move || {
                        let result = node.send_output_encoded(output_id, parameters, encoded);
                        (node, result)
                    });
                    let result;
                    (node, result) = await_send_watching_for_stop(
                        &mut send,
                        &mut events,
                        &mut pending,
                        &mut operator_channels,
                    )
                    .await
                    .wrap_err("failed to wait for send_output task")?;
                    result.wrap_err("failed to send node output")?;
                }
            },
            RuntimeEvent::Event(Event::Stop(cause)) => {
                forward_stop(&mut operator_channels, &cause).await;
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
            RuntimeEvent::Event(Event::Error(err)) => {
                eyre::bail!("received error event: {err}");
            }
            RuntimeEvent::Event(other) => {
                tracing::warn!("received unknown event `{other:?}`");
            }
        }
    }

    mem::drop(events);

    Ok(())
}

/// Forward `Stop` to every operator and close their event channels.
///
/// Diagnostic (dora-rs/dora#2742): a logged "received Stop" with no matching
/// "forwarded Stop" means the forward blocked on a full operator channel — i.e.
/// the operator is parked in its own `on_event` and never draining. The `warn!`
/// level is deliberate: the runtime's default stdout filter is `warn`
/// (`with_stdout("warn", …)`), so anything quieter never reaches a nightly log —
/// which is what made the original wedge invisible.
async fn forward_stop(
    operator_channels: &mut HashMap<OperatorId, flume::Sender<Event>>,
    cause: &StopCause,
) {
    tracing::warn!(
        "runtime received Stop; forwarding to {} operator(s) (dora-rs/dora#2742 diagnostic)",
        operator_channels.len()
    );
    for (id, channel) in operator_channels.drain() {
        let _ = channel.send_async(Event::Stop(cause.clone())).await;
        tracing::warn!("forwarded Stop to operator `{id}` (dora-rs/dora#2742 diagnostic)");
    }
}

/// Wait for an in-flight output send without going deaf to `Stop`
/// (dora-rs/dora#2742).
///
/// The main loop is the only consumer of the merged operator/daemon event
/// stream, so awaiting a send inline meant the node could not observe `Stop` —
/// and could not let its operators start winding down — until the send
/// returned. This keeps consuming the stream: `Stop` is forwarded to the
/// operators immediately, everything else is buffered for the caller to handle,
/// in order, once the send completes.
///
/// This does **not** rescue a send that never returns: the node still owes the
/// daemon an exit and will be force-killed at the grace period. Bailing out of a
/// wedged send is not possible from here — the `DoraNode` lives inside the
/// blocking task, and `Runtime::drop` waits forever for `spawn_blocking` work,
/// so a real escape needs `shutdown_background()` at the `main()` layer.
async fn await_send_watching_for_stop<S, T>(
    send: &mut tokio::task::JoinHandle<T>,
    events: &mut S,
    pending: &mut VecDeque<RuntimeEvent>,
    operator_channels: &mut HashMap<OperatorId, flume::Sender<Event>>,
) -> Result<T, tokio::task::JoinError>
where
    S: Stream<Item = RuntimeEvent> + Unpin,
{
    /// Stop pulling events into memory once this many are buffered. Each
    /// buffered `Event::Input` pins its payload (a mapped shared-memory region
    /// above the zero-copy threshold), so this is deliberately small: it only
    /// has to cover the handful of events that can arrive while one send is in
    /// flight. Beyond it the events simply stay in the stream.
    const MAX_BUFFERED: usize = 4;

    let mut stop_seen = false;
    loop {
        // Once `Stop` is forwarded there is nothing left to watch for, so stop
        // consuming and just wait the send out.
        let watching = !stop_seen && pending.len() < MAX_BUFFERED;
        tokio::select! {
            biased;
            joined = &mut *send => return joined,
            event = events.next(), if watching => {
                match event {
                    Some(RuntimeEvent::Event(Event::Stop(cause))) => {
                        forward_stop(operator_channels, &cause).await;
                        stop_seen = true;
                    }
                    Some(other) => pending.push_back(other),
                    // The stream ended, so no `Stop` can arrive.
                    None => stop_seen = true,
                }
            }
        }
    }
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

#[cfg(test)]
mod stop_responsiveness_tests {
    use super::*;
    use futures::stream;

    fn operator_channel() -> (
        HashMap<OperatorId, flume::Sender<Event>>,
        flume::Receiver<Event>,
    ) {
        let (tx, rx) = flume::unbounded();
        let mut channels = HashMap::new();
        channels.insert(OperatorId::from("op".to_string()), tx);
        (channels, rx)
    }

    /// The ordinary case: nothing else happens, the send completes, and no
    /// event is buffered.
    #[tokio::test]
    async fn a_send_that_completes_is_returned() {
        let mut send = tokio::spawn(async { 42 });
        let mut events = stream::empty::<RuntimeEvent>();
        let mut pending = VecDeque::new();
        let (mut channels, _rx) = operator_channel();

        let joined =
            await_send_watching_for_stop(&mut send, &mut events, &mut pending, &mut channels)
                .await
                .expect("joined");

        assert_eq!(joined, 42);
        assert!(pending.is_empty());
    }

    /// The #2742 behaviour: a `Stop` arriving mid-send reaches the operator
    /// without waiting for the send, and a non-`Stop` event is kept for the
    /// caller rather than dropped or reordered.
    #[tokio::test]
    async fn stop_reaches_the_operator_while_a_send_is_in_flight() {
        let (unblock_tx, unblock_rx) = tokio::sync::oneshot::channel::<()>();
        let mut send = tokio::spawn(async move {
            let _ = unblock_rx.await;
            7
        });
        let mut events = stream::iter(vec![
            RuntimeEvent::Event(Event::Reload { operator_id: None }),
            RuntimeEvent::Event(Event::Stop(StopCause::Manual)),
        ]);
        let mut pending = VecDeque::new();
        let (mut channels, rx) = operator_channel();

        // Release the send only after the operator has been told to stop, which
        // is only possible if the forward happened while the send was pending.
        tokio::spawn(async move {
            while rx.is_empty() {
                tokio::task::yield_now().await;
            }
            assert!(matches!(rx.recv_async().await, Ok(Event::Stop(_))));
            let _ = unblock_tx.send(());
        });

        let joined = tokio::time::timeout(
            Duration::from_secs(5),
            await_send_watching_for_stop(&mut send, &mut events, &mut pending, &mut channels),
        )
        .await
        .expect("must not hang")
        .expect("joined");

        assert_eq!(joined, 7);
        assert_eq!(
            pending.len(),
            1,
            "the non-Stop event must be kept, not lost"
        );
        assert!(
            channels.is_empty(),
            "forwarding Stop closes the operator channels"
        );
    }

    /// Buffering is bounded: a flood of events during a send does not pull the
    /// whole stream into memory.
    #[tokio::test]
    async fn buffering_during_a_send_is_bounded() {
        let (unblock_tx, unblock_rx) = tokio::sync::oneshot::channel::<()>();
        let mut send = tokio::spawn(async move {
            let _ = unblock_rx.await;
            0
        });
        let flood = (0..1000)
            .map(|_| RuntimeEvent::Event(Event::Reload { operator_id: None }))
            .collect::<Vec<_>>();
        let mut events = stream::iter(flood);
        let mut pending = VecDeque::new();
        let (mut channels, _rx) = operator_channel();

        tokio::spawn(async move {
            tokio::time::sleep(Duration::from_millis(50)).await;
            let _ = unblock_tx.send(());
        });

        await_send_watching_for_stop(&mut send, &mut events, &mut pending, &mut channels)
            .await
            .expect("joined");

        assert!(
            pending.len() <= 4,
            "buffered {} events, expected the cap to hold",
            pending.len()
        );
    }
}
