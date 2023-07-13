#[allow(unused_imports)]
use log::{debug, error, info, warn};
use futures::{future, StreamExt};
use serde::{Deserialize, Serialize};
use ros2_client::{AService, Context, Message, Node, NodeOptions, ServiceMapping};
use rustdds::{
  policy::{self, Deadline, Lifespan},
  Duration, QosPolicies, QosPolicyBuilder,
};

// This is an example / test program.
// Test this against minimal_client found in
// https://github.com/ros2/examples/blob/master/rclpy/services/minimal_client/examples_rclpy_minimal_client/client.py
// or
// % ros2 run examples_rclpy_minimal_client client
// or
// % ros2 run examples_rclcpp_minimal_client client_main

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AddTwoIntsRequest {
  pub a: i64,
  pub b: i64,
}
impl Message for AddTwoIntsRequest {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AddTwoIntsResponse {
  pub sum: i64,
}
impl Message for AddTwoIntsResponse {}

fn main() {
  pretty_env_logger::init();

  debug!(">>> ros2_service starting...");
  let mut node = create_node();
  let service_qos = create_qos();

  println!(">>> ros2_service node started");

  let server = node
    .create_server::<AService<AddTwoIntsRequest, AddTwoIntsResponse>>(
      ServiceMapping::Enhanced,
      "/add_two_ints",
      "example_interfaces::srv::dds_::AddTwoInts_Request_", // req type name
      "example_interfaces::srv::dds_::AddTwoInts_Response_", // resp type name
      service_qos.clone(),
      service_qos.clone(),
    )
    .unwrap();

  println!(">>> ros2_service server created");

  let server_stream = server.receive_request_stream().then(|result| async {
    match result {
      Ok((req_id, req)) => {
        println!("request: {} + {}", req.a, req.b);
        let resp = AddTwoIntsResponse { sum: req.a + req.b };
        let sr = server.async_send_response(req_id, resp).await;
        if let Err(e) = sr {
          println!("Send error {:?}", e);
        }
      }
      Err(e) => println!("Receive request error: {:?}", e),
    }
  });

  // run it!
  smol::block_on(async { server_stream.for_each(|_result| future::ready(())).await });
} // main

fn create_qos() -> QosPolicies {
  let service_qos: QosPolicies = {
    QosPolicyBuilder::new()
      .history(policy::History::KeepLast { depth: 10 })
      .reliability(policy::Reliability::Reliable {
        max_blocking_time: Duration::from_millis(100),
      })
      .durability(policy::Durability::Volatile)
      .deadline(Deadline(Duration::DURATION_INFINITE))
      .lifespan(Lifespan {
        duration: Duration::DURATION_INFINITE,
      })
      .liveliness(policy::Liveliness::Automatic {
        lease_duration: Duration::DURATION_INFINITE,
      })
      .build()
  };
  service_qos
}

fn create_node() -> Node {
  let context = Context::new().unwrap();
  let node = context
    .new_node(
      "rustdds_server",
      "/rustdds",
      NodeOptions::new().enable_rosout(true),
    )
    .unwrap();
  node
}
