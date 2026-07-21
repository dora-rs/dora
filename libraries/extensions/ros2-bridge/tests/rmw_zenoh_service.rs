#![cfg(feature = "rmw-zenoh")]

use std::time::{Duration, Instant};

use dora_ros2_bridge::transport::{
    RequestId,
    zenoh::{
        graph::GraphCache,
        service::{PendingRequests, ServiceError, request_id_from_attachment, wait_for_service},
    },
};

fn id(sequence_number: i64, gid: u8) -> RequestId {
    RequestId {
        sequence_number,
        client_gid: [gid; 16],
    }
}

#[test]
fn service_request_cdr_and_request_ids_are_preserved() {
    let mut pending = PendingRequests::new(64, Duration::from_secs(30));
    let cdr = vec![0, 1, 0, 0, 42, 0, 0, 0];
    pending
        .insert(id(7, 1), cdr.clone(), Duration::ZERO)
        .unwrap();
    pending.insert(id(7, 2), vec![9], Duration::ZERO).unwrap();
    assert_eq!(pending.take(id(7, 2), Duration::ZERO).unwrap(), vec![9]);
    assert_eq!(pending.take(id(7, 1), Duration::ZERO).unwrap(), cdr);
}

#[test]
fn service_pending_is_bounded_and_expires_with_fake_clock() {
    let mut pending = PendingRequests::new(64, Duration::from_secs(30));
    for sequence in 0..64 {
        pending
            .insert(id(sequence, 3), sequence, Duration::ZERO)
            .unwrap();
    }
    assert!(matches!(
        pending.insert(id(64, 3), 64, Duration::ZERO),
        Err(ServiceError::PendingLimit { limit: 64 })
    ));
    assert!(matches!(
        pending.take(id(0, 3), Duration::from_secs(30)),
        Err(ServiceError::RequestExpired)
    ));
    assert!(pending.is_empty());
}

#[test]
fn malformed_or_missing_attachments_are_rejected() {
    assert!(matches!(
        request_id_from_attachment(None),
        Err(ServiceError::MalformedAttachment)
    ));
    assert!(matches!(
        request_id_from_attachment(Some(&[1, 2, 3])),
        Err(ServiceError::MalformedAttachment)
    ));
}

#[tokio::test]
async fn wait_for_service_handles_exact_match_timeout_and_shutdown() {
    let graph = GraphCache::new(7);
    let deadline = Instant::now() + Duration::from_millis(20);
    assert!(matches!(
        wait_for_service(
            &graph,
            "/add",
            "example_interfaces::srv::dds_::AddTwoInts_",
            "RIHS01_hash",
            "qos",
            deadline
        )
        .await,
        Err(ServiceError::Timeout)
    ));

    use dora_ros2_bridge::transport::zenoh::keyexpr::{EntityKind, LivelinessKey, TopicToken};
    let service = LivelinessKey::endpoint(
        7,
        "zid",
        "nid",
        "eid",
        EntityKind::Service,
        "/",
        "/",
        "node",
        TopicToken {
            name: "/add".into(),
            type_name: "example_interfaces::srv::dds_::AddTwoInts_".into(),
            type_hash: "RIHS01_hash".into(),
            qos: "qos".into(),
        },
    )
    .unwrap();
    graph.apply_put(service.as_str()).unwrap();
    wait_for_service(
        &graph,
        "/add",
        "example_interfaces::srv::dds_::AddTwoInts_",
        "RIHS01_hash",
        "qos",
        Instant::now(),
    )
    .await
    .unwrap();

    let closed = GraphCache::new(7);
    closed.shutdown();
    assert!(matches!(
        wait_for_service(
            &closed,
            "/add",
            "type",
            "hash",
            "qos",
            Instant::now() + Duration::from_secs(1)
        )
        .await,
        Err(ServiceError::TransportClosed)
    ));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn service_concurrent_calls_correlate_out_of_order_and_surface_failures() {
    use dora_ros2_bridge::transport::zenoh::service::{RawServiceClient, RawServiceServer};

    let session = zenoh::open(zenoh::Config::default()).await.unwrap();
    let server =
        RawServiceServer::declare(&session, "dora/test/service", 64, Duration::from_secs(30))
            .await
            .unwrap();
    let client = RawServiceClient::declare(&session, "dora/test/service", [4; 16], 64)
        .await
        .unwrap();

    let worker = tokio::spawn(async move {
        let mut requests = Vec::new();
        for _ in 0..16 {
            requests.push(server.recv().await.unwrap());
        }
        for request in requests.into_iter().rev() {
            server.reply(request.id, &request.payload).await.unwrap();
        }
    });
    let calls = (0u8..16).map(|value| client.call(vec![0, 1, 0, 0, value], Duration::from_secs(2)));
    let replies = futures::future::join_all(calls).await;
    for (value, reply) in (0u8..16).zip(replies) {
        assert_eq!(reply.unwrap(), vec![0, 1, 0, 0, value]);
    }
    worker.await.unwrap();

    drop(client);
    let absent = RawServiceClient::declare(&session, "dora/test/absent", [5; 16], 64)
        .await
        .unwrap();
    assert!(matches!(
        absent.call(vec![1], Duration::from_millis(20)).await,
        Err(ServiceError::Timeout)
    ));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn multiple_complete_servers_remote_errors_and_disappearance_are_observable() {
    use dora_ros2_bridge::transport::zenoh::service::{RawServiceClient, RawServiceServer};
    use std::sync::Arc;

    let session = zenoh::open(zenoh::Config::default()).await.unwrap();
    let first =
        RawServiceServer::declare(&session, "dora/test/multiple", 64, Duration::from_secs(30))
            .await
            .unwrap();
    let second =
        RawServiceServer::declare(&session, "dora/test/multiple", 64, Duration::from_secs(30))
            .await
            .unwrap();
    let client = Arc::new(
        RawServiceClient::declare(&session, "dora/test/multiple", [6; 16], 64)
            .await
            .unwrap(),
    );
    let call_client = client.clone();
    let call = tokio::spawn(async move { call_client.call(vec![8], Duration::from_secs(2)).await });
    let (request_a, request_b) = tokio::join!(first.recv(), second.recv());
    let request_a = request_a.unwrap();
    let request_b = request_b.unwrap();
    assert_eq!(request_a.id, request_b.id);
    first.reject(request_a.id, "server refused").await.unwrap();
    second.reply(request_b.id, &[9]).await.unwrap();
    assert!(
        matches!(call.await.unwrap(), Err(ServiceError::Remote(message)) if message == "server refused")
    );

    drop(first);
    drop(second);
    assert!(matches!(
        client.call(vec![1], Duration::from_millis(20)).await,
        Err(ServiceError::Timeout)
    ));
}
