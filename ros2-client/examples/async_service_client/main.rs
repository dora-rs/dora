use std::time::Duration;

#[allow(unused_imports)]
use log::{debug, error, info, warn};
use futures::{FutureExt as StdFutureExt, StreamExt};
use smol::future::FutureExt;
use serde::{Deserialize, Serialize};
use ros2_client::{AService, Context, Message, Node, NodeOptions, ServiceMapping};
use rustdds::{policy, QosPolicies, QosPolicyBuilder};

// Test / demo program of ROS2 services, client side.
//
// To set up a server from ROS2:
// % ros2 run examples_rclcpp_minimal_service service_main
// or
// % ros2 run examples_rclpy_minimal_service service
//
// Then run this example.

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

  // Set Ctrl-C handler
  let (stop_sender, stop_receiver) = smol::channel::bounded(2);
  ctrlc::set_handler(move || {
    // We will send two stop coammnds, one for reader, the other for writer.
    stop_sender.send_blocking(()).unwrap_or(());
    stop_sender.send_blocking(()).unwrap_or(());
    // ignore errors, as we are quitting anyway
  })
  .expect("Error setting Ctrl-C handler");
  println!("Press Ctrl-C to quit.");

  debug!(">>> ros2_service starting...");
  let mut node = create_node();
  let service_qos = create_qos();

  debug!(">>> ros2_service node started");

  let client = node
    .create_client::<AService<AddTwoIntsRequest, AddTwoIntsResponse>>(
      ServiceMapping::Enhanced,
      "/add_two_ints",
      "example_interfaces::srv::dds_::AddTwoInts_Request_", // req type name
      "example_interfaces::srv::dds_::AddTwoInts_Response_", // resp type name
      service_qos.clone(),
      service_qos.clone(),
    )
    .unwrap();

  debug!(">>> ros2_service client created");

  let mut request_generator = 0;

  let main_loop = async {
    let mut run = true;
    let mut stop = stop_receiver.recv().fuse();
    let mut tick_stream = futures::StreamExt::fuse(smol::Timer::interval(Duration::from_secs(2)));

    while run {
      futures::select! {
        _ = stop => run = false,
        _tick = tick_stream.select_next_some() => {
          request_generator += 3;
          let a = request_generator % 5;
          let b = request_generator % 7;
          match client.async_send_request(AddTwoIntsRequest { a, b }).await {
            Ok(req_id) => {
              println!(">>> request sent a={} b={}, {:?}", a, b, req_id.sequence_number);
              match
                client.async_receive_response(req_id)
                  .or(async {
                        smol::Timer::after(Duration::from_secs(2)).await;
                        Err(rustdds::dds::Error::MustBlock )
                      }).await

              {
                Ok(response) => {
                  println!("<<< response: {:?}", response);
                }
                Err(e) => println!("<<< response error {:?}", e),
              }
            }
            Err(e) => println!(">>> request sending error {:?}", e),
          }
        }
      } // select!
    } // while
    debug!("main loop done");
  };

  // run it!
  smol::block_on(main_loop);
}

fn create_qos() -> QosPolicies {
  let service_qos: QosPolicies = {
    QosPolicyBuilder::new()
      .reliability(policy::Reliability::Reliable {
        max_blocking_time: rustdds::Duration::from_millis(100),
      })
      .history(policy::History::KeepLast { depth: 1 })
      .build()
  };
  service_qos
}

fn create_node() -> Node {
  let context = Context::new().unwrap();
  let node = context
    .new_node(
      "rustdds_client",
      "/rustdds",
      NodeOptions::new().enable_rosout(true),
    )
    .unwrap();
  node
}
