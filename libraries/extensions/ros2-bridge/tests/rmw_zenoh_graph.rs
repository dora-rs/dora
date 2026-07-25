#![cfg(feature = "rmw-zenoh")]

use std::time::Duration;

use dora_ros2_bridge::transport::zenoh::{
    ConfigSource, EndpointToken, EntityIdAllocator, NodeToken,
    graph::{GraphCache, GraphError},
    keyexpr::{EntityKind, LivelinessKey, TopicToken},
    resolve_config_source,
};
use std::sync::{
    Arc,
    atomic::{AtomicUsize, Ordering},
};

fn node(domain: usize, id: &str) -> LivelinessKey {
    LivelinessKey::node(domain, "zid", "nid", id, "/", "/robot", "driver").unwrap()
}

fn endpoint(domain: usize, id: &str, kind: EntityKind) -> LivelinessKey {
    LivelinessKey::endpoint(
        domain,
        "zid",
        "nid",
        id,
        kind,
        "/",
        "/robot",
        "driver",
        TopicToken {
            name: "/robot/add_two_ints".into(),
            type_name: "example_interfaces::srv::dds_::AddTwoInts_".into(),
            type_hash: "TypeHashNotSupported".into(),
            qos: "2::,1:,:,:,,,".into(),
        },
    )
    .unwrap()
}

#[test]
fn graph_initial_query_is_sorted_and_domain_isolated() {
    let graph = GraphCache::new(42);
    graph
        .apply_put(endpoint(0, "ignored", EntityKind::Service).as_str())
        .unwrap();
    graph
        .apply_put(endpoint(42, "second", EntityKind::Service).as_str())
        .unwrap();
    graph.apply_put(node(42, "first").as_str()).unwrap();
    let snapshot = graph.snapshot();
    assert_eq!(snapshot.entities.len(), 2);
    assert!(snapshot.entities[0].key < snapshot.entities[1].key);
}

#[test]
fn graph_delete_is_idempotent_and_duplicate_ids_remain_distinct() {
    let graph = GraphCache::new(7);
    let first = endpoint(7, "same", EntityKind::Service);
    let second = LivelinessKey::endpoint(
        7,
        "other-zid",
        "nid",
        "same",
        EntityKind::Service,
        "/",
        "/robot",
        "driver",
        first.topic.clone().unwrap(),
    )
    .unwrap();
    assert!(graph.apply_put(first.as_str()).unwrap().changed());
    assert!(graph.apply_put(second.as_str()).unwrap().changed());
    assert!(!graph.apply_put(first.as_str()).unwrap().changed());
    assert!(graph.apply_delete(first.as_str()).unwrap().changed());
    assert!(!graph.apply_delete(first.as_str()).unwrap().changed());
    assert_eq!(graph.snapshot().entities.len(), 1);
}

#[test]
fn graph_accepts_endpoint_before_node_and_rejects_malformed_tokens() {
    let graph = GraphCache::new(7);
    graph
        .apply_put(endpoint(7, "service", EntityKind::Service).as_str())
        .unwrap();
    graph.apply_put(node(7, "node").as_str()).unwrap();
    assert_eq!(graph.snapshot().entities.len(), 2);
    assert!(matches!(
        graph.apply_put("not/a/token"),
        Err(GraphError::MalformedToken(_))
    ));
}

#[tokio::test]
async fn graph_waiters_wake_once_on_change_and_on_shutdown() {
    let graph = GraphCache::new(7);
    let generation = graph.snapshot().generation;
    let waiter = graph.wait_for_change(generation);
    graph.apply_put(node(7, "node").as_str()).unwrap();
    assert!(
        tokio::time::timeout(Duration::from_secs(1), waiter)
            .await
            .unwrap()
            .is_ok()
    );

    let generation = graph.snapshot().generation;
    let waiter = graph.wait_for_change(generation);
    graph.shutdown();
    assert!(matches!(
        tokio::time::timeout(Duration::from_secs(1), waiter)
            .await
            .unwrap(),
        Err(GraphError::Closed)
    ));
}

#[test]
fn graph_service_matching_checks_complete_identity() {
    let graph = GraphCache::new(7);
    graph
        .apply_put(endpoint(7, "service", EntityKind::Service).as_str())
        .unwrap();
    assert_eq!(
        graph
            .matching_services(
                "/robot/add_two_ints",
                "example_interfaces::srv::dds_::AddTwoInts_",
                "TypeHashNotSupported",
                "2::,1:,:,:,,,"
            )
            .len(),
        1
    );
    assert!(
        graph
            .matching_services(
                "/other",
                "example_interfaces::srv::dds_::AddTwoInts_",
                "TypeHashNotSupported",
                "2::,1:,:,:,,,"
            )
            .is_empty()
    );
}

#[test]
fn config_precedence_is_explicit_then_environment_then_embedded() {
    assert_eq!(
        resolve_config_source(Some("explicit.json5"), Some("environment.json5")).unwrap(),
        ConfigSource::Explicit("explicit.json5".into())
    );
    assert_eq!(
        resolve_config_source(None, Some("environment.json5")).unwrap(),
        ConfigSource::Environment("environment.json5".into())
    );
    assert_eq!(
        resolve_config_source(None, None).unwrap(),
        ConfigSource::EmbeddedDefault
    );
    assert!(resolve_config_source(Some(""), None).is_err());
}

#[test]
fn entity_ids_are_stable_monotonic_and_gids_are_16_bytes() {
    let allocator = EntityIdAllocator::new([0x2a; 8]);
    let first = allocator.allocate();
    let second = allocator.allocate();
    assert_eq!(first.numeric + 1, second.numeric);
    assert_ne!(first.gid, second.gid);
    assert_eq!(&first.gid[..8], &[0x2a; 8]);
}

#[test]
fn raii_tokens_drop_their_guards_exactly_once() {
    struct Guard(Arc<AtomicUsize>);
    impl Drop for Guard {
        fn drop(&mut self) {
            self.0.fetch_add(1, Ordering::SeqCst);
        }
    }
    let drops = Arc::new(AtomicUsize::new(0));
    {
        let _node = NodeToken::from_guard("node", Guard(drops.clone()));
        let _endpoint = EndpointToken::from_guard("endpoint", Guard(drops.clone()));
        assert_eq!(drops.load(Ordering::SeqCst), 0);
    }
    assert_eq!(drops.load(Ordering::SeqCst), 2);
}
