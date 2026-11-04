#![cfg(feature = "rmw-zenoh")]

use std::{sync::Arc, time::Duration};

use dora_ros2_bridge::transport::{
    Durability, History, Publisher as TransportPublisher, Reliability, Ros2Qos,
    Subscription as TransportSubscription, TransportError,
    zenoh::{
        Context, ContextOptions,
        attachment::Attachment,
        graph::GraphSnapshot,
        keyexpr::{EntityKind, TopicToken},
        pubsub::{
            NodePublisher, NodeSubscription, PubSubError, PublisherMode, PublisherState,
            RawPublisher, RawSubscription, SubscriptionIngress,
        },
    },
};

/// Liveliness-token declaration/undeclaration propagates through the Zenoh
/// session asynchronously, so the graph cache updates a beat after
/// `declare`/`drop` return. Await the graph reaching the expected state
/// (event-driven via `wait_for_change`) instead of asserting the snapshot
/// synchronously, which races and flakes.
async fn wait_for_graph(context: &Context, predicate: impl Fn(&GraphSnapshot) -> bool) {
    tokio::time::timeout(Duration::from_secs(5), async {
        loop {
            let snapshot = context.graph.snapshot();
            if predicate(&snapshot) {
                break;
            }
            context
                .graph
                .wait_for_change(snapshot.generation)
                .await
                .expect("graph closed while waiting for a change");
        }
    })
    .await
    .expect("timed out waiting for the graph to reach the expected state");
}

#[test]
fn publisher_sequence_starts_at_one_and_metadata_is_preserved() {
    let state = PublisherState::new([0x2a; 16]);
    let first = state.metadata_at(100).unwrap();
    let second = state.metadata_at(-1).unwrap();
    assert_eq!((first.sequence_number, second.sequence_number), (1, 2));
    assert_eq!(first.publisher_gid, [0x2a; 16]);
    assert_eq!(second.source_timestamp_ns, -1);
}

#[test]
fn subscriber_parses_attachment_before_payload_and_bounds_queue() {
    let ingress =
        SubscriptionIngress::new(1, 8, Arc::new(|bytes: &[u8]| Ok(bytes.to_vec()))).unwrap();
    let attachment = Attachment {
        sequence_number: 7,
        source_timestamp_ns: 11,
        gid: [3; 16],
    }
    .encode()
    .unwrap();
    assert!(ingress.ingest(&attachment, b"first").unwrap());
    assert!(!ingress.ingest(&attachment, b"second").unwrap());
    let (payload, metadata) = ingress.try_recv().unwrap();
    assert_eq!(payload, b"first");
    assert_eq!(metadata.sequence_number, 7);
    assert_eq!(ingress.stats().dropped_samples, 1);
}

#[test]
fn subscriber_keep_last_drops_oldest_sample_on_full_queue() {
    let qos = Ros2Qos {
        history: History::KeepLast { depth: 2 },
        ..Ros2Qos::default()
    };
    let ingress = SubscriptionIngress::new_with_qos(
        2,
        8,
        Arc::new(|bytes: &[u8]| Ok(bytes.to_vec())),
        Some(&qos),
    )
    .unwrap();

    let make_attachment = |seq: i64| {
        Attachment {
            sequence_number: seq,
            source_timestamp_ns: 100 + seq,
            gid: [3; 16],
        }
        .encode()
        .unwrap()
    };

    assert!(ingress.ingest(&make_attachment(1), b"1").unwrap());
    assert!(ingress.ingest(&make_attachment(2), b"2").unwrap());
    assert!(ingress.ingest(&make_attachment(3), b"3").unwrap());
    assert!(ingress.ingest(&make_attachment(4), b"4").unwrap());
    assert!(ingress.ingest(&make_attachment(5), b"5").unwrap());

    let (payload1, meta1) = ingress.try_recv().unwrap();
    let (payload2, meta2) = ingress.try_recv().unwrap();
    assert!(ingress.try_recv().is_err());

    assert_eq!(payload1, b"4");
    assert_eq!(meta1.sequence_number, 4);
    assert_eq!(payload2, b"5");
    assert_eq!(meta2.sequence_number, 5);
    assert_eq!(ingress.stats().dropped_samples, 3);
}

#[test]
fn subscriber_keep_all_drops_newest_sample_on_full_queue() {
    let qos = Ros2Qos {
        history: History::KeepAll,
        ..Ros2Qos::default()
    };
    let ingress = SubscriptionIngress::new_with_qos(
        2,
        8,
        Arc::new(|bytes: &[u8]| Ok(bytes.to_vec())),
        Some(&qos),
    )
    .unwrap();

    let make_attachment = |seq: i64| {
        Attachment {
            sequence_number: seq,
            source_timestamp_ns: 100 + seq,
            gid: [3; 16],
        }
        .encode()
        .unwrap()
    };

    assert!(ingress.ingest(&make_attachment(1), b"1").unwrap());
    assert!(ingress.ingest(&make_attachment(2), b"2").unwrap());
    assert!(!ingress.ingest(&make_attachment(3), b"3").unwrap());
    assert!(!ingress.ingest(&make_attachment(4), b"4").unwrap());
    assert!(!ingress.ingest(&make_attachment(5), b"5").unwrap());

    let (payload1, meta1) = ingress.try_recv().unwrap();
    let (payload2, meta2) = ingress.try_recv().unwrap();
    assert!(ingress.try_recv().is_err());

    assert_eq!(payload1, b"1");
    assert_eq!(meta1.sequence_number, 1);
    assert_eq!(payload2, b"2");
    assert_eq!(meta2.sequence_number, 2);
    assert_eq!(ingress.stats().dropped_samples, 3);
}

fn ingress_with_history(capacity: usize, history: History) -> SubscriptionIngress<Vec<u8>> {
    let qos = Ros2Qos {
        history,
        ..Ros2Qos::default()
    };
    SubscriptionIngress::new_with_qos(
        capacity,
        8,
        Arc::new(|bytes: &[u8]| Ok(bytes.to_vec())),
        Some(&qos),
    )
    .unwrap()
}

fn ingest_sequence(ingress: &SubscriptionIngress<Vec<u8>>, range: std::ops::RangeInclusive<i64>) {
    for seq in range {
        let attachment = Attachment {
            sequence_number: seq,
            source_timestamp_ns: 100 + seq,
            gid: [3; 16],
        }
        .encode()
        .unwrap();
        ingress
            .ingest(&attachment, seq.to_string().as_bytes())
            .unwrap();
    }
}

fn drain_sequence_numbers(ingress: &SubscriptionIngress<Vec<u8>>) -> Vec<i64> {
    std::iter::from_fn(|| ingress.try_recv().ok())
        .map(|(payload, meta)| {
            assert_eq!(payload, meta.sequence_number.to_string().as_bytes());
            meta.sequence_number
        })
        .collect()
}

/// A KEEP_LAST queue holds `depth` samples even when the caller's `capacity`
/// is larger: the bridge passes a fixed capacity (64), and `depth: 1` must not
/// let 63 stale samples pile up behind the newest one.
#[test]
fn subscriber_keep_last_queue_is_bounded_by_depth_not_capacity() {
    let ingress = ingress_with_history(8, History::KeepLast { depth: 2 });
    ingest_sequence(&ingress, 1..=5);
    assert_eq!(drain_sequence_numbers(&ingress), vec![4, 5]);
    assert_eq!(ingress.stats().dropped_samples, 3);
}

/// A depth deeper than `capacity` is still capped by `capacity`.
#[test]
fn subscriber_keep_last_queue_is_capped_by_capacity() {
    let ingress = ingress_with_history(2, History::KeepLast { depth: 8 });
    ingest_sequence(&ingress, 1..=5);
    assert_eq!(drain_sequence_numbers(&ingress), vec![4, 5]);
    assert_eq!(ingress.stats().dropped_samples, 3);
}

/// KEEP_ALL has no depth, so it is bounded by `capacity` alone and drops the
/// newest sample only once all `capacity` slots are taken.
#[test]
fn subscriber_keep_all_queue_is_bounded_by_capacity() {
    let ingress = ingress_with_history(8, History::KeepAll);
    ingest_sequence(&ingress, 1..=5);
    assert_eq!(ingress.stats().dropped_samples, 0);
    ingest_sequence(&ingress, 6..=10);
    assert_eq!(
        drain_sequence_numbers(&ingress),
        (1..=8).collect::<Vec<_>>()
    );
    assert_eq!(ingress.stats().dropped_samples, 2);
}

#[test]
fn subscriber_rejects_non_positive_keep_last_depth() {
    for depth in [0, -1] {
        let qos = Ros2Qos {
            history: History::KeepLast { depth },
            ..Ros2Qos::default()
        };
        let result = SubscriptionIngress::new_with_qos(
            8,
            8,
            Arc::new(|bytes: &[u8]| Ok(bytes.to_vec())),
            Some(&qos),
        );
        assert!(
            matches!(result, Err(PubSubError::InvalidDepth(d)) if d == depth),
            "depth {depth} must be rejected"
        );
    }
}

#[test]
fn subscriber_rejects_malformed_attachment_and_payload_before_decode() {
    let ingress =
        SubscriptionIngress::new(2, 4, Arc::new(|bytes: &[u8]| Ok(bytes.to_vec()))).unwrap();
    assert!(ingress.ingest(&[0], b"ok").is_err());
    let attachment = Attachment {
        sequence_number: 1,
        source_timestamp_ns: 1,
        gid: [0; 16],
    }
    .encode()
    .unwrap();
    assert!(ingress.ingest(&attachment, b"oversized").is_err());
    assert_eq!(ingress.stats().malformed_samples, 2);
}

#[test]
fn qos_selects_reliability_history_and_transient_local_behavior() {
    let best_effort = PublisherMode::from_qos(&Ros2Qos::default()).unwrap();
    assert!(matches!(
        best_effort,
        PublisherMode::Standard {
            block: false,
            depth: Some(1),
            transient_local: false
        }
    ));
    let reliable = PublisherMode::from_qos(&Ros2Qos {
        reliability: Reliability::Reliable {
            max_blocking_time: Duration::from_millis(100),
        },
        durability: Durability::TransientLocal,
        history: History::KeepLast { depth: 10 },
        ..Ros2Qos::default()
    })
    .unwrap();
    assert!(matches!(
        reliable,
        PublisherMode::Advanced {
            block: true,
            depth: Some(10),
            transient_local: true
        }
    ));
    let keep_all = PublisherMode::from_qos(&Ros2Qos {
        history: History::KeepAll,
        ..Ros2Qos::default()
    })
    .unwrap();
    assert!(matches!(
        keep_all,
        PublisherMode::Standard {
            block: true,
            depth: None,
            ..
        }
    ));
}

#[test]
fn unsupported_qos_events_are_typed_errors() {
    assert!(matches!(
        dora_ros2_bridge::transport::zenoh::pubsub::register_qos_event("deadline"),
        Err(TransportError::UnsupportedQosEvent { event: "deadline" })
    ));
    assert!(matches!(
        dora_ros2_bridge::transport::zenoh::pubsub::register_qos_event("liveliness"),
        Err(TransportError::UnsupportedQosEvent {
            event: "liveliness"
        })
    ));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn two_sessions_preserve_order_payload_and_metadata() {
    let publisher_session = zenoh::open(zenoh::Config::default()).await.unwrap();
    let subscriber_session = zenoh::open(zenoh::Config::default()).await.unwrap();
    let subscription = RawSubscription::declare(
        &subscriber_session,
        "42/dora/test",
        4,
        1024,
        Arc::new(|bytes: &[u8]| Ok(bytes.to_vec())),
    )
    .await
    .unwrap();
    let publisher = RawPublisher::declare(
        &publisher_session,
        "42/dora/test",
        [9; 16],
        &Ros2Qos::default(),
    )
    .await
    .unwrap();
    assert!(!publisher.is_advanced());
    let reliable_publisher = RawPublisher::declare(
        &publisher_session,
        "42/dora/reliable",
        [8; 16],
        &Ros2Qos {
            reliability: Reliability::Reliable {
                max_blocking_time: Duration::from_millis(100),
            },
            ..Ros2Qos::default()
        },
    )
    .await
    .unwrap();
    assert!(!reliable_publisher.is_advanced());
    tokio::time::sleep(Duration::from_millis(500)).await;
    publisher.publish_at(b"one", 100).await.unwrap();
    publisher.publish_at(b"two", 101).await.unwrap();
    publisher.publish(b"three").await.unwrap();
    let first = tokio::time::timeout(Duration::from_secs(3), subscription.recv_async())
        .await
        .unwrap()
        .unwrap();
    let second = tokio::time::timeout(Duration::from_secs(3), subscription.recv_async())
        .await
        .unwrap()
        .unwrap();
    let third = tokio::time::timeout(Duration::from_secs(3), subscription.recv_async())
        .await
        .unwrap()
        .unwrap();
    assert_eq!((first.0, first.1.sequence_number), (b"one".to_vec(), 1));
    assert_eq!((second.0, second.1.sequence_number), (b"two".to_vec(), 2));
    assert_eq!((third.0, third.1.sequence_number), (b"three".to_vec(), 3));
    let publisher: TransportPublisher<()> = TransportPublisher::Zenoh(Box::new(reliable_publisher));
    let subscription: TransportSubscription<()> =
        TransportSubscription::Zenoh(Box::new(subscription));
    assert!(matches!(publisher, TransportPublisher::Zenoh(_)));
    assert!(matches!(subscription, TransportSubscription::Zenoh(_)));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn transient_local_delivers_history_to_late_joiner() {
    let publisher_session = zenoh::open(zenoh::Config::default()).await.unwrap();
    let subscriber_session = zenoh::open(zenoh::Config::default()).await.unwrap();
    let qos = Ros2Qos {
        durability: Durability::TransientLocal,
        history: History::KeepLast { depth: 1 },
        ..Ros2Qos::default()
    };
    let publisher = RawPublisher::declare(&publisher_session, "42/dora/transient", [7; 16], &qos)
        .await
        .unwrap();
    publisher.publish_at(b"retained", 99).await.unwrap();
    tokio::time::sleep(Duration::from_millis(250)).await;
    let subscription = RawSubscription::declare_with_qos(
        &subscriber_session,
        "42/dora/transient",
        2,
        1024,
        Arc::new(|bytes: &[u8]| Ok(bytes.to_vec())),
        &qos,
    )
    .await
    .unwrap();
    assert!(subscription.is_advanced());
    let sample = tokio::time::timeout(Duration::from_secs(5), subscription.recv_async())
        .await
        .unwrap()
        .unwrap();
    assert_eq!(sample.0, b"retained");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn live_subscription_isolates_keys_counts_malformed_and_shuts_down() {
    let session = zenoh::open(zenoh::Config::default()).await.unwrap();
    let subscription = RawSubscription::declare(
        &session,
        "42/robot/ns/topic",
        1,
        4,
        Arc::new(|bytes: &[u8]| Ok(bytes.to_vec())),
    )
    .await
    .unwrap();
    let valid = Attachment {
        sequence_number: 1,
        source_timestamp_ns: 1,
        gid: [1; 16],
    }
    .encode()
    .unwrap();
    session
        .put("0/robot/ns/topic", b"none".to_vec())
        .attachment(valid.clone())
        .await
        .unwrap();
    assert!(
        tokio::time::timeout(Duration::from_millis(250), subscription.recv_async())
            .await
            .is_err()
    );
    session
        .put("42/robot/ns/topic", b"data".to_vec())
        .attachment(vec![0])
        .await
        .unwrap();
    tokio::time::sleep(Duration::from_millis(100)).await;
    assert_eq!(subscription.stats().malformed_samples, 1);
    drop(subscription);
    session
        .put("42/robot/ns/topic", b"data".to_vec())
        .attachment(valid)
        .await
        .unwrap();
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn node_bound_entities_track_graph_and_drop_tokens() {
    let context = Context::open(ContextOptions {
        domain_id: 42,
        config_uri: None,
    })
    .await
    .unwrap();
    let node = context
        .create_node("zid", "nid", "/", "/robot", "node")
        .await
        .unwrap();
    let topic = TopicToken {
        name: "/robot/topic".into(),
        type_name: "std_msgs::msg::dds_::String_".into(),
        type_hash: "TypeHashNotSupported".into(),
        qos: "2::,1:,:,:,,,".into(),
    };
    let key = "42/robot/topic/std_msgs::msg::dds_::String_/TypeHashNotSupported";
    let publisher = NodePublisher::declare(&node, key, topic.clone(), &Ros2Qos::default())
        .await
        .unwrap();
    let subscription = NodeSubscription::declare(
        &node,
        key,
        topic,
        &Ros2Qos::default(),
        2,
        1024,
        Arc::new(|bytes: &[u8]| Ok(bytes.to_vec())),
    )
    .await
    .unwrap();
    wait_for_graph(&context, |snapshot| {
        snapshot
            .entities
            .iter()
            .filter(|entity| {
                matches!(
                    entity.token.kind,
                    EntityKind::Publisher | EntityKind::Subscription
                )
            })
            .count()
            == 2
    })
    .await;
    drop(subscription);
    drop(publisher);
    wait_for_graph(&context, |snapshot| {
        snapshot
            .entities
            .iter()
            .all(|entity| entity.token.kind == EntityKind::Node)
    })
    .await;
}

/// Peer config linked only over loopback TCP, with multicast scouting off, so
/// two sessions find each other without multicast (absent in containers and on
/// some CI runners).
fn loopback_config(listen: Option<&str>, connect: Option<&str>) -> zenoh::Config {
    let mut config = zenoh::Config::default();
    config
        .insert_json5("scouting/multicast/enabled", "false")
        .unwrap();
    config
        .insert_json5(
            "listen/endpoints",
            &format!(
                "[{}]",
                listen.map(|e| format!("\"{e}\"")).unwrap_or_default()
            ),
        )
        .unwrap();
    if let Some(endpoint) = connect {
        config
            .insert_json5("connect/endpoints", &format!("[\"{endpoint}\"]"))
            .unwrap();
    }
    config
}

/// Open two sessions linked directly over loopback: the first listens on an
/// OS-assigned port and the second connects to the locator it actually bound.
async fn loopback_session_pair() -> (zenoh::Session, zenoh::Session) {
    let listener = zenoh::open(loopback_config(Some("tcp/127.0.0.1:0"), None))
        .await
        .unwrap();
    let locator = listener
        .info()
        .locators()
        .await
        .into_iter()
        .map(|locator| locator.as_str().to_owned())
        .find(|locator| locator.starts_with("tcp/127.0.0.1:"))
        .expect("listener session bound no loopback TCP locator");
    let connector = zenoh::open(loopback_config(None, Some(&locator)))
        .await
        .unwrap();
    (listener, connector)
}

async fn wait_until(what: &str, mut condition: impl AsyncFnMut() -> bool) {
    tokio::time::timeout(Duration::from_secs(5), async {
        while !condition().await {
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    })
    .await
    .unwrap_or_else(|_| panic!("timed out waiting for {what}"));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn raw_subscription_keep_last_drops_oldest_sample_on_full_queue() {
    const KEY: &str = "42/dora/keep_last_drop";
    let (publisher_session, subscriber_session) = loopback_session_pair().await;
    // Reliable so the publisher cannot drop a sample under congestion; every
    // drop counted below is then the subscription's KEEP_LAST eviction.
    let qos = Ros2Qos {
        reliability: Reliability::Reliable {
            max_blocking_time: Duration::from_millis(100),
        },
        history: History::KeepLast { depth: 2 },
        ..Ros2Qos::default()
    };
    // Capacity well above the depth, as the bridge passes: the queue must
    // still hold only `depth` samples.
    let subscription = RawSubscription::declare_with_qos(
        &subscriber_session,
        KEY,
        64,
        1024,
        Arc::new(|bytes: &[u8]| Ok(bytes.to_vec())),
        &qos,
    )
    .await
    .unwrap();
    let publisher = RawPublisher::declare(&publisher_session, KEY, [1; 16], &qos)
        .await
        .unwrap();

    // Publish only once the publisher's session routes the key to the remote
    // subscriber; a sample put before that would be lost, not queued.
    let probe = publisher_session.declare_publisher(KEY).await.unwrap();
    wait_until("the subscription to match the publisher", async || {
        probe.matching_status().await.unwrap().matching()
    })
    .await;
    drop(probe);

    for i in 1..=5 {
        publisher
            .publish_at(format!("{i}").as_bytes(), 100 + i)
            .await
            .unwrap();
    }

    // With nothing read yet, a depth-2 queue evicts one sample for each one
    // past the second, so three drops means all five samples were ingested.
    wait_until("all five samples to be ingested", async || {
        subscription.stats().dropped_samples >= 3
    })
    .await;

    let first = tokio::time::timeout(Duration::from_secs(3), subscription.recv_async())
        .await
        .unwrap()
        .unwrap();
    let second = tokio::time::timeout(Duration::from_secs(3), subscription.recv_async())
        .await
        .unwrap()
        .unwrap();

    assert_eq!((first.0, first.1.sequence_number), (b"4".to_vec(), 4));
    assert_eq!((second.0, second.1.sequence_number), (b"5".to_vec(), 5));
    assert!(subscription.try_recv().unwrap().is_none());
    assert_eq!(subscription.stats().dropped_samples, 3);
}
