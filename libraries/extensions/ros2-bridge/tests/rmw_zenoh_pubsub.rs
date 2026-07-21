#![cfg(feature = "rmw-zenoh")]

use std::{sync::Arc, time::Duration};

use dora_ros2_bridge::transport::{
    Durability, History, Publisher as TransportPublisher, Reliability, Ros2Qos,
    Subscription as TransportSubscription, TransportError,
    zenoh::{
        Context, ContextOptions,
        attachment::Attachment,
        keyexpr::{EntityKind, TopicToken},
        pubsub::{
            NodePublisher, NodeSubscription, PublisherMode, PublisherState, RawPublisher,
            RawSubscription, SubscriptionIngress,
        },
    },
};

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
    assert_eq!(
        context
            .graph
            .snapshot()
            .entities
            .iter()
            .filter(|entity| matches!(
                entity.token.kind,
                EntityKind::Publisher | EntityKind::Subscription
            ))
            .count(),
        2
    );
    drop(subscription);
    drop(publisher);
    assert!(
        context
            .graph
            .snapshot()
            .entities
            .iter()
            .all(|entity| entity.token.kind == EntityKind::Node)
    );
}
