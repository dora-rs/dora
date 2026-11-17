//! Regression test for DDS services under `ServiceMapping::Cyclone` (the
//! mapping the bridge selects for `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`).
//!
//! `ros2-client` 0.10 decoded the body of every Cyclone-mapped request and
//! response from the 16-byte CycloneHeader instead of the payload after it, so
//! dora service/action servers and clients exchanged garbage with CycloneDDS
//! peers. The request here is exactly 16 bytes, the size of that header, so a
//! regression shows up as a wrong `sum` rather than only as a decode error.
//!
//! Pure RustDDS: needs no ROS installation.

use std::time::{Duration, Instant};

use dora_ros2_bridge::ros2_client::{
    self, Context, ContextOptions, Message, Name, NodeName, NodeOptions, QosProfile,
    ServiceMapping, ServiceTypeName,
};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct AddRequest {
    a: i64,
    b: i64,
}
impl Message for AddRequest {}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct AddResponse {
    sum: i64,
}
impl Message for AddResponse {}

/// A domain no other test in this crate uses, so parallel tests don't match.
const DOMAIN_ID: u16 = 87;
const SERVICE: &str = "dora_cyclone_mapping_regression";

fn node(name: &str) -> ros2_client::Node {
    Context::with_options(ContextOptions::new().domain_id(DOMAIN_ID))
        .expect("DDS context")
        .new_node(NodeName::new("/", name).unwrap(), NodeOptions::new())
        .expect("ROS 2 node")
}

#[test]
fn cyclone_mapped_service_round_trips_payload() {
    let service_name = Name::new("/", SERVICE).unwrap();
    let service_type = ServiceTypeName::new("dora_test", "AddTwoInts");
    let qos = QosProfile::publisher_default();

    let mut server_node = node("cyclone_server");
    let server = server_node
        .create_server::<AddRequest, AddResponse>(
            ServiceMapping::Cyclone,
            &service_name,
            &service_type,
            qos.clone(),
            qos.clone(),
        )
        .expect("service server");

    let mut client_node = node("cyclone_client");
    let client = client_node
        .create_client::<AddRequest, AddResponse>(
            ServiceMapping::Cyclone,
            &service_name,
            &service_type,
            qos.clone(),
            qos,
        )
        .expect("service client");

    let deadline = Instant::now() + Duration::from_secs(30);
    let mut sent = Vec::new();
    let mut server_saw = None;
    let mut last_send = Instant::now() - Duration::from_secs(1);

    // Discovery takes a moment, so resend until the round trip completes.
    let (response_id, response) = loop {
        assert!(
            Instant::now() < deadline,
            "no response within 30 s; server saw {server_saw:?}"
        );
        if last_send.elapsed() >= Duration::from_millis(500) {
            if let Ok(id) = client.send_request(AddRequest { a: 41, b: 1 }) {
                sent.push(id);
            }
            last_send = Instant::now();
        }
        while let Some((id, request)) = server.receive_request().expect("server read") {
            server_saw = Some(request.clone());
            server
                .send_response(
                    id,
                    AddResponse {
                        sum: request.a + request.b,
                    },
                )
                .expect("server write");
        }
        if let Some(reply) = client.receive_response().expect("client read") {
            break reply;
        }
        std::thread::sleep(Duration::from_millis(20));
    };

    assert_eq!(server_saw, Some(AddRequest { a: 41, b: 1 }));
    assert_eq!(response, AddResponse { sum: 42 });
    assert!(
        sent.contains(&response_id),
        "response id {response_id:?} does not match any sent request {sent:?}"
    );
}
