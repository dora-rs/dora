//! Late-join validation for the schema-cache subtopic (the W3 design): a
//! subscriber that joins *after* the producer published its schema must still
//! receive it, via the zenoh-ext `AdvancedPublisher` cache + `AdvancedSubscriber`
//! history query. This mirrors `node::schema_publisher` /
//! `event_stream::declare_schema_subscriber` exactly (depth-1 cache,
//! SequenceNumber sequencing via `sample_miss_detection` so no session
//! timestamping is needed, `publisher_detection` + `detect_late_publishers`).
//!
//! `#[ignore]` because it opens real loopback zenoh sessions on a fixed port:
//! run on demand with `cargo test -p dora-node-api --test zenoh_schema_cache --
//! --ignored`. Kept out of the default suite to avoid port/timing flakiness in
//! parallel CI. The cold-start path is covered end-to-end by the example smoke.

use std::time::Duration;

use zenoh::Wait;
use zenoh::qos::CongestionControl;
use zenoh_ext::{
    AdvancedPublisherBuilderExt, AdvancedSubscriberBuilderExt, CacheConfig, HistoryConfig,
    MissDetectionConfig,
};

fn config(listen: &[&str], connect: &[&str]) -> zenoh::Config {
    let mut c = zenoh::Config::default();
    c.insert_json5("scouting/multicast/enabled", "false")
        .unwrap();
    let as_json = |eps: &[&str]| {
        format!(
            "[{}]",
            eps.iter()
                .map(|s| format!("\"{s}\""))
                .collect::<Vec<_>>()
                .join(",")
        )
    };
    if !listen.is_empty() {
        c.insert_json5("listen/endpoints", &as_json(listen))
            .unwrap();
        c.insert_json5("listen/exit_on_failure", "false").unwrap();
    }
    if !connect.is_empty() {
        c.insert_json5("connect/endpoints", &as_json(connect))
            .unwrap();
    }
    c
}

#[test]
#[ignore = "opens real loopback zenoh sessions; run with --ignored"]
fn advanced_pub_cache_serves_late_joining_subscriber() {
    let endpoint = "tcp/127.0.0.1:17471";
    let key = "dora/test/schema-cache/output/@schema";
    let schema = b"PRETEND-IPC-SCHEMA-MESSAGE";

    // Producer joins first and publishes the schema, then keeps its session
    // (and the cache) alive for the rest of the test.
    let pub_session = zenoh::open(config(&[endpoint], &[])).wait().unwrap();
    let publisher = pub_session
        .declare_publisher(key)
        .congestion_control(CongestionControl::Block)
        .sample_miss_detection(MissDetectionConfig::default())
        .cache(CacheConfig::default())
        .publisher_detection()
        .wait()
        .unwrap();
    publisher.put(schema.as_slice()).wait().unwrap();

    // Let the publish + cache settle before anyone is listening.
    std::thread::sleep(Duration::from_millis(300));

    // Subscriber joins LATER — it missed the live publish and must recover the
    // schema from the publisher's cache through its history query.
    let (tx, rx) = std::sync::mpsc::channel();
    let sub_session = zenoh::open(config(&[], &[endpoint])).wait().unwrap();
    let _subscriber = sub_session
        .declare_subscriber(key)
        .history(HistoryConfig::default().detect_late_publishers())
        .callback(move |sample| {
            let _ = tx.send(sample.payload().to_bytes().to_vec());
        })
        .wait()
        .unwrap();

    let got = rx
        .recv_timeout(Duration::from_secs(10))
        .expect("late-joining subscriber must receive the cached schema");
    assert_eq!(
        got,
        schema.as_slice(),
        "cached schema bytes must round-trip to the late joiner"
    );
}
