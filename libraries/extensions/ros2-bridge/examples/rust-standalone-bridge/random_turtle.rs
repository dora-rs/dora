//! Instructions:
//!
//! - Source the ROS2 setup files (e.g. `source /opt/ros/iron/setup.bash`)
//! - Run `ros2 run turtlesim turtlesim_node`
//! - Open a second terminal and source the ROS2 setup files again.
//! - Run this example to move the turtle in random directions: `cargo run --example random_turtle`

use std::time::Duration;

use dora_ros2_bridge::{
    self,
    geometry_msgs::msg::{Twist, Vector3},
    ros2_client::{self, ros2, NodeOptions},
    rustdds::{self, policy},
    turtlesim::srv::SetPen_Request,
};
use futures::FutureExt;

fn main() {
    let ros_context = ros2_client::Context::new().unwrap();

    let mut ros_node = ros_context
        .new_node(
            "turtle_teleop", // name
            "/ros2_demo",    // namespace
            NodeOptions::new().enable_rosout(true),
        )
        .unwrap();

    let topic_qos: rustdds::QosPolicies = {
        rustdds::QosPolicyBuilder::new()
            .durability(policy::Durability::Volatile)
            .liveliness(policy::Liveliness::Automatic {
                lease_duration: ros2::Duration::DURATION_INFINITE,
            })
            .reliability(policy::Reliability::Reliable {
                max_blocking_time: ros2::Duration::from_millis(100),
            })
            .history(policy::History::KeepLast { depth: 1 })
            .build()
    };

    let turtle_cmd_vel_topic = ros_node
        .create_topic(
            "/turtle1/cmd_vel",
            String::from("geometry_msgs::msg::dds_::Twist_"),
            &topic_qos,
        )
        .unwrap();

    // The point here is to publish Twist for the turtle
    let turtle_cmd_vel_writer = ros_node
        .create_publisher::<Twist>(&turtle_cmd_vel_topic, None)
        .unwrap();

    let turtle_pose_topic = ros_node
        .create_topic(
            "/turtle1/pose",
            String::from("turtlesim::msg::dds_::Pose_"),
            &Default::default(),
        )
        .unwrap();
    let turtle_pose_reader = ros_node
        .create_subscription::<dora_ros2_bridge::turtlesim::msg::Pose>(&turtle_pose_topic, None)
        .unwrap();

    for _ in 0..100 {
        let direction = Twist {
            linear: Vector3 {
                x: rand::random::<f64>() + 1.0,
                ..Default::default()
            },
            angular: Vector3 {
                z: (rand::random::<f64>() - 0.5) * 5.0,
                ..Default::default()
            },
        };
        println!("sending {direction:?}");
        turtle_cmd_vel_writer.publish(direction).unwrap();
        std::thread::sleep(Duration::from_millis(500));

        while let Some(Ok((pose, _info))) = turtle_pose_reader.async_take().now_or_never() {
            println!("{pose:?}");
        }
    }
}

#[derive(Debug)]
pub enum RosCommand {
    StopEventLoop,
    TurtleCmdVel {
        turtle_id: i32,
        twist: Twist,
    },
    Reset,
    SetPen(SetPen_Request),
    Spawn(String),
    Kill(String),
    RotateAbsolute {
        heading: f32,
    },
    #[allow(non_camel_case_types)]
    RotateAbsolute_Cancel,
}
