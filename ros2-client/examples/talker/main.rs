use log::error;
use mio::{Events, Poll, PollOpt, Ready, Token};
use mio_extras::timer;
use ros2_client::{Context, Node, NodeOptions};
use rustdds::{
  policy::{self, Deadline, Lifespan},
  Duration, QosPolicies, QosPolicyBuilder,
};
//use core::cmp::min;

// Simple demo program.
// Test this against ROS2 "listener" demo node.
//
// Listener start in ROS2:
// % ros2 run demo_nodes_py listener
//
// Then run this example program to talk to the listener.

fn main() {
  // Here is a fixed path, so this example must be started from
  // RustDDS main directory
  log4rs::init_file("examples/talker/log4rs.yaml", Default::default()).unwrap();

  let mut node = create_node();
  let topic_qos = create_qos();

  let chatter_topic = node
    .create_topic(
      "/chatter",
      String::from("std_msgs::msg::dds_::String_"),
      &topic_qos,
    )
    .unwrap();
  let chatter_publisher = node
    .create_publisher::<String>(&chatter_topic, None)
    .unwrap();

  let mut talk_timer: timer::Timer<()> = timer::Builder::default().build();

  let poll = Poll::new().unwrap();

  poll
    .register(&talk_timer, Token(1), Ready::readable(), PollOpt::edge())
    .unwrap();

  talk_timer.set_timeout(std::time::Duration::from_secs(2), ());

  let mut events = Events::with_capacity(8);
  let mut count = 0;

  let filler: String =
    "All work and no play makes ROS a dull boy. All play and no work makes RTPS a mere toy. "
      .repeat(2);
  // Change repeat count to e.g. 100 to test data fragmentation.

  loop {
    poll.poll(&mut events, None).unwrap();

    for event in events.iter() {
      match event.token() {
        Token(1) => {
          count += 1;
          let message = format!("count={} {}", count, filler);
          println!("Talking, count={} len={}", count, message.len());
          chatter_publisher
            .publish(message)
            .unwrap_or_else(|e| error!("publish failed: {:?}", e));
          talk_timer.set_timeout(std::time::Duration::from_secs(2), ());
        }
        _ => println!(">>> Unknown poll token {:?}", event.token()),
      } // match
    } // for
  } // loop
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
      "rustdds_listener",
      "/rustdds",
      NodeOptions::new().enable_rosout(true),
    )
    .unwrap();
  node
}
