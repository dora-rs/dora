#![deny(clippy::all)]

#[cfg(not(unix))]
fn main() {
  println!("This example only works on a unix based system");
}

use std::time::Duration;

#[cfg(unix)]
use termion::raw::*;
#[allow(unused_imports)]
use log::{debug, error, info, warn};
use serde::{Deserialize, Serialize};
use mio_06::{Events, Poll, PollOpt, Ready, Token};
use mio_extras::channel as mio_channel;
use rustdds::{
  policy::*,
  ros2::{NodeOptions, RosParticipant},
  *,
};
#[cfg(unix)]
use ui::{RosCommand, UiController};

// modules
#[cfg(unix)]
mod ui;

const TURTLE_CMD_VEL_READER_TOKEN: Token = Token(1);
const ROS2_COMMAND_TOKEN: Token = Token(2);
const TURTLE_POSE_READER_TOKEN: Token = Token(3);

// This corresponds to ROS2 message type
// https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Twist.msg
//
// The struct definition must have a layout corresponding to the
// ROS2 msg definition to get compatible serialization.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Twist {
  pub linear: Vector3,
  pub angular: Vector3,
}

// https://docs.ros2.org/foxy/api/turtlesim/msg/Pose.html
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Pose {
  pub x: f32,
  pub y: f32,
  pub theta: f32,
  pub linear_velocity: f32,
  pub angular_velocity: f32,
}

// This corresponds to ROS2 message type
// https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Vector3.msg
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Vector3 {
  pub x: f64,
  pub y: f64,
  pub z: f64,
}

impl Vector3 {
  pub const ZERO: Vector3 = Vector3 {
    x: 0.0,
    y: 0.0,
    z: 0.0,
  };
}

#[cfg(unix)]
fn main() {
  // Here is a fixed path, so this example must be started from
  // RustDDS main directory
  log4rs::init_file("examples/turtle_teleop/log4rs.yaml", Default::default()).unwrap();

  let (command_sender, command_receiver) = mio_channel::sync_channel::<RosCommand>(10);
  let (readback_sender, readback_receiver) = mio_channel::sync_channel(10);
  let (pose_sender, pose_receiver) = mio_channel::sync_channel(10);

  // For some strange reason the ROS2 messaging event loop is in a separate thread
  // and we talk to it using (mio) mpsc channels.
  let jhandle = std::thread::Builder::new()
    .name("ros2_loop".into())
    .spawn(move || ros2_loop(command_receiver, readback_sender, pose_sender))
    .unwrap();

  // From termion docs:
  // "A terminal restorer, which keeps the previous state of the terminal,
  // and restores it, when dropped.
  // Restoring will entirely bring back the old TTY state."
  // So the point of _stdout_restorer is that it will restore the TTY back to
  // its original cooked mode when the variable is dropped.
  let _stdout_restorer = std::io::stdout().into_raw_mode().unwrap();

  // UI loop, which is in the main thread
  let mut main_control = UiController::new(
    std::io::stdout(),
    command_sender,
    readback_receiver,
    pose_receiver,
  );
  main_control.start();

  jhandle.join().unwrap(); // wait until threads exit.

  // need to wait a bit for cleanup, because drop is not waited for join
  std::thread::sleep(Duration::from_millis(10));
}

#[cfg(unix)]
fn ros2_loop(
  command_receiver: mio_channel::Receiver<RosCommand>,
  readback_sender: mio_channel::SyncSender<Twist>,
  pose_sender: mio_channel::SyncSender<Pose>,
) {
  info!("ros2_loop");

  let qos: QosPolicies = {
    QosPolicyBuilder::new()
      .durability(Durability::Volatile)
      .liveliness(Liveliness::Automatic {
        lease_duration: rustdds::Duration::DURATION_INFINITE,
      })
      .reliability(Reliability::Reliable {
        max_blocking_time: rustdds::Duration::from_millis(100),
      })
      .history(History::KeepLast { depth: 10 })
      .build()
  };

  let mut ros_participant = RosParticipant::new().unwrap();

  let mut ros_node = ros_participant
    .new_ros_node(
      "turtle_teleop",         // name
      "/ros2_demo",            // namespace
      NodeOptions::new(false), // enable rosout
    )
    .unwrap();

  let turtle_cmd_vel_topic = ros_node
    .create_ros_topic(
      "/turtle1/cmd_vel",
      String::from("geometry_msgs::msg::dds_::Twist_"),
      &qos,
      TopicKind::NoKey,
    )
    .unwrap();

  // The point here is to publish Twist for the turtle
  let turtle_cmd_vel_writer = ros_node
    .create_ros_no_key_publisher::<Twist, CDRSerializerAdapter<Twist>>(&turtle_cmd_vel_topic, None)
    .unwrap();

  // But here is how to read it also, if anyone is interested.
  // This should show what is the turtle command in case someone else is
  // also issuing commands, i.e. there are two turtle controllers running.
  let mut turtle_cmd_vel_reader = ros_node
    .create_ros_no_key_subscriber::<Twist, CDRDeserializerAdapter<_>>(&turtle_cmd_vel_topic, None)
    .unwrap();

  let turtle_pose_topic = ros_node
    .create_ros_topic(
      "/turtle1/pose",
      String::from("turtlesim::msg::dds_::Pose_"),
      &qos,
      TopicKind::NoKey,
    )
    .unwrap();
  let mut turtle_pose_reader = ros_node
    .create_ros_no_key_subscriber::<Pose, CDRDeserializerAdapter<_>>(&turtle_pose_topic, None)
    .unwrap();

  let poll = Poll::new().unwrap();

  poll
    .register(
      &command_receiver,
      ROS2_COMMAND_TOKEN,
      Ready::readable(),
      PollOpt::edge(),
    )
    .unwrap();

  poll
    .register(
      &turtle_cmd_vel_reader,
      TURTLE_CMD_VEL_READER_TOKEN,
      Ready::readable(),
      PollOpt::edge(),
    )
    .unwrap();
  poll
    .register(
      &turtle_pose_reader,
      TURTLE_POSE_READER_TOKEN,
      Ready::readable(),
      PollOpt::edge(),
    )
    .unwrap();

  info!("Entering event_loop");
  'event_loop: loop {
    let mut events = Events::with_capacity(100);
    poll.poll(&mut events, None).unwrap();

    for event in events.iter() {
      match event.token() {
        ROS2_COMMAND_TOKEN => {
          while let Ok(command) = command_receiver.try_recv() {
            match command {
              RosCommand::StopEventLoop => {
                info!("Stopping main event loop");
                ros_participant.clear();
                break 'event_loop;
              }
              RosCommand::TurtleCmdVel { twist } => {
                match turtle_cmd_vel_writer.write(twist.clone(), None) {
                  Ok(_) => {
                    info!("Wrote to ROS2 {:?}", twist);
                  }
                  Err(e) => {
                    error!("Failed to write to turtle writer. {e:?}");
                    ros_node.clear_node();
                    return;
                  }
                }
              }
            };
          }
        }
        TURTLE_CMD_VEL_READER_TOKEN => {
          while let Ok(Some(twist)) = turtle_cmd_vel_reader.take_next_sample() {
            readback_sender.send(twist.value().clone()).unwrap();
          }
        }
        TURTLE_POSE_READER_TOKEN => {
          while let Ok(Some(pose)) = turtle_pose_reader.take_next_sample() {
            pose_sender.send(pose.value().clone()).unwrap();
          }
        }
        _ => {
          error!("Unknown poll token {:?}", event.token())
        }
      } // match
    } // for
  }
}
