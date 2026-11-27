//! Reading `dora topic` data straight from zenoh (dora-rs/dora#4025).
//!
//! The coordinator answers a direct subscription with one
//! [`ZenohTopicSource`] per daemon running a requested output. For each, this
//! opens a zenoh *client* session to the daemon's endpoint, which forwards its
//! own nodes' samples to the clients linked to it, and subscribes there to the
//! source's outputs. Each sample is rebuilt the way the daemon rebuilds the
//! relayed ones ([`prepare_debug_topic_frame`]) and handed to the subscription
//! as the same `Timestamped<InterDaemonEvent::Output>` bytes a relayed frame
//! carries, so `topic echo`/`hz`/`info` can't tell the two apart.

use std::{sync::Arc, time::Duration};

use dora_core::{
    debug_topic::{DebugSchemaCache, prepare_debug_topic_frame, retain_published_schema},
    topics::{zenoh_output_publish_topic, zenoh_output_schema_topic},
    uhlc::HLC,
};
use dora_message::{
    common::Timestamped,
    coordinator_to_cli::ZenohTopicSource,
    daemon_to_daemon::InterDaemonEvent,
    id::{DataId, NodeId},
    metadata::Metadata,
};
use eyre::{Context, eyre};
use uuid::Uuid;
use zenoh::{handlers::FifoChannelHandler, pubsub::Subscriber, sample::Sample};

use crate::ws_client::TopicSubscriber;

/// How long to wait for a daemon's zenoh endpoint before using the relay.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(3);

/// Env var that turns direct subscriptions off (`off`, `0`, `false`, `no`),
/// for a network where the endpoints are reachable but should not be used.
const DIRECT_ENV: &str = "DORA_TOPIC_DIRECT";

/// Whether to ask the coordinator for a direct subscription.
pub(crate) fn enabled() -> bool {
    !std::env::var(DIRECT_ENV).is_ok_and(|v| {
        matches!(
            v.trim().to_ascii_lowercase().as_str(),
            "off" | "0" | "false" | "no"
        )
    })
}

/// Subscribe to every source's outputs and start forwarding them to
/// `subscriber`.
///
/// Fails, handing `subscriber` back for the relay, if a source's endpoint
/// can't be reached or a subscription can't be declared; nothing is left
/// running then. Once it succeeds, the forwarding tasks run until the
/// receiver is dropped.
pub(crate) async fn subscribe(
    dataflow_id: Uuid,
    sources: Vec<ZenohTopicSource>,
    subscriber: TopicSubscriber,
) -> Result<(), (eyre::Report, TopicSubscriber)> {
    let outputs = match open_sources(dataflow_id, sources).await {
        Ok(outputs) => outputs,
        Err(err) => return Err((err, subscriber)),
    };
    let subscriber = Arc::new(subscriber);
    let clock = Arc::new(HLC::default());
    for output in outputs {
        tokio::spawn(output.forward(dataflow_id, subscriber.clone(), clock.clone()));
    }
    Ok(())
}

/// One subscribed output, with what keeps its subscriptions alive.
struct DirectOutput {
    node_id: NodeId,
    output_id: DataId,
    data: Subscriber<FifoChannelHandler<Sample>>,
    schema_cache: DebugSchemaCache,
    /// Dropping it would undeclare the `@schema` subscription.
    _schema: zenoh_ext::AdvancedSubscriber<()>,
    /// Shared by the outputs of one source; the session closes with the last.
    _session: zenoh::Session,
}

async fn open_sources(
    dataflow_id: Uuid,
    sources: Vec<ZenohTopicSource>,
) -> eyre::Result<Vec<DirectOutput>> {
    let mut outputs = Vec::new();
    for source in sources {
        let session = open_client_session(&source.endpoint)
            .await
            .wrap_err_with(|| format!("failed to reach zenoh endpoint `{}`", source.endpoint))?;
        for (node_id, output_id) in source.topics {
            outputs.push(subscribe_output(&session, dataflow_id, node_id, output_id).await?);
        }
    }
    Ok(outputs)
}

/// A zenoh session that links to `endpoint` only: client mode, so it opens no
/// listener and joins no mesh, and no multicast scouting.
async fn open_client_session(endpoint: &str) -> eyre::Result<zenoh::Session> {
    let mut config = zenoh::Config::default();
    let insert = |config: &mut zenoh::Config, key: &str, value: &str| {
        config
            .insert_json5(key, value)
            .map_err(|e| eyre!("invalid zenoh config `{key}`: {e}"))
    };
    insert(&mut config, "mode", r#""client""#)?;
    // Serialized rather than interpolated: the endpoint came over the wire.
    insert(
        &mut config,
        "connect/endpoints",
        &serde_json::to_string(&[endpoint])?,
    )?;
    insert(
        &mut config,
        "connect/timeout_ms",
        &CONNECT_TIMEOUT.as_millis().to_string(),
    )?;
    insert(&mut config, "scouting/multicast/enabled", "false")?;
    // zenoh gives up after `connect/timeout_ms`; this only guards against it
    // not doing so.
    tokio::time::timeout(CONNECT_TIMEOUT * 2, zenoh::open(config))
        .await
        .map_err(|_| eyre!("timed out"))?
        .map_err(|e| eyre!(e))
}

async fn subscribe_output(
    session: &zenoh::Session,
    dataflow_id: Uuid,
    node_id: NodeId,
    output_id: DataId,
) -> eyre::Result<DirectOutput> {
    use zenoh_ext::{AdvancedSubscriberBuilderExt, HistoryConfig};

    let topic = zenoh_output_publish_topic(dataflow_id, &node_id, &output_id);
    let data = session
        .declare_subscriber(topic.clone())
        .await
        .map_err(|e| eyre!(e))
        .wrap_err_with(|| format!("failed to subscribe to `{topic}`"))?;

    // Small outputs are schema-once batches: the schema is published once on
    // the `@schema` key. Fetch it with history, as the daemon's relay does, so
    // the batches can be rebuilt even when the producer started earlier.
    let schema_cache: DebugSchemaCache = Default::default();
    let schema_topic = zenoh_output_schema_topic(dataflow_id, &node_id, &output_id);
    let schema = {
        let cache = schema_cache.clone();
        session
            .declare_subscriber(schema_topic.clone())
            .history(HistoryConfig::default().detect_late_publishers())
            .callback(move |sample| {
                retain_published_schema(
                    &mut cache.lock().unwrap_or_else(|e| e.into_inner()),
                    &sample.payload().to_bytes(),
                );
            })
            .await
            .map_err(|e| eyre!(e))
            .wrap_err_with(|| format!("failed to subscribe to `{schema_topic}`"))?
    };

    Ok(DirectOutput {
        node_id,
        output_id,
        data,
        schema_cache,
        _schema: schema,
        _session: session.clone(),
    })
}

impl DirectOutput {
    async fn forward(self, dataflow_id: Uuid, subscriber: Arc<TopicSubscriber>, clock: Arc<HLC>) {
        while let Ok(sample) = self.data.recv_async().await {
            let Some(frame) = self.frame(dataflow_id, &sample, &clock) else {
                continue;
            };
            if subscriber.offer(&frame).is_err() {
                break; // the receiver is gone
            }
        }
    }

    /// The relayed frame for `sample`, or `None` to skip it.
    fn frame(&self, dataflow_id: Uuid, sample: &Sample, clock: &HLC) -> Option<Vec<u8>> {
        // A node publishes the raw payload with its encoded metadata as the
        // attachment (see `DoraNode::zenoh_publish`).
        let metadata = sample
            .attachment()
            .and_then(|a| dora_message::decode::<Metadata>(&a.to_bytes()).ok())?;
        // Startup-handshake markers ride the data key with an empty payload.
        if metadata.is_startup_marker() {
            return None;
        }
        let payload = sample.payload().to_bytes();
        let (metadata, data) = prepare_debug_topic_frame(
            &mut self.schema_cache.lock().unwrap_or_else(|e| e.into_inner()),
            true,
            metadata,
            &payload,
        )?;
        let event = Timestamped {
            inner: InterDaemonEvent::Output {
                dataflow_id,
                node_id: self.node_id.clone(),
                output_id: self.output_id.clone(),
                metadata,
                data: Some(aligned_vec::AVec::from_slice(128, &data)),
            },
            timestamp: clock.new_timestamp(),
        };
        match event.serialize() {
            Ok(frame) => Some(frame),
            Err(err) => {
                tracing::debug!("failed to encode a direct topic frame: {err:?}");
                None
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ws_client::{TopicBufferLimits, topic_channel};

    /// An endpoint that can't be reached fails within the connect timeout and
    /// hands the subscriber back, still connected to its receiver, so the
    /// caller can relay into the same receiver instead.
    // zenoh refuses tokio's current-thread scheduler.
    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn an_unreachable_source_hands_the_subscriber_back() {
        let (subscriber, receiver) = topic_channel(TopicBufferLimits::LIVE);
        // Nothing listens on port 1.
        let sources = vec![ZenohTopicSource::new(
            "tcp/127.0.0.1:1".to_string(),
            vec![("camera".to_string().into(), "image".to_string().into())],
        )];

        let started = std::time::Instant::now();
        let (err, subscriber) = subscribe(Uuid::nil(), sources, subscriber)
            .await
            .expect_err("an unreachable endpoint must fail");

        assert!(
            started.elapsed() < CONNECT_TIMEOUT * 2,
            "{:?}",
            started.elapsed()
        );
        assert!(
            format!("{err:#}").contains("tcp/127.0.0.1:1"),
            "the error names the endpoint: {err:#}"
        );
        assert!(subscriber.offer(b"frame").is_ok());
        assert_eq!(receiver.recv().unwrap().unwrap(), b"frame");
    }

    #[test]
    fn direct_subscriptions_can_be_turned_off() {
        for (value, expected) in [
            (None, true),
            (Some("on"), true),
            (Some("off"), false),
            (Some(" OFF "), false),
            (Some("0"), false),
            (Some("false"), false),
            (Some("no"), false),
        ] {
            // SAFETY: this is the only test that touches `DORA_TOPIC_DIRECT`,
            // and it doesn't run concurrently with itself.
            unsafe {
                match value {
                    Some(v) => std::env::set_var(DIRECT_ENV, v),
                    None => std::env::remove_var(DIRECT_ENV),
                }
            }
            assert_eq!(enabled(), expected, "{value:?}");
        }
        unsafe { std::env::remove_var(DIRECT_ENV) };
    }
}
