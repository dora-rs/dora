use dora_node_api::{
    self,
    dora_core::config::DataId,
    merged::{MergeExternal, MergedEvent},
    DoraNode, Event,
};
use dora_ros2_bridge::{
    geometry_msgs::msg::{Twist, Vector3},
    ros2_client::{self, ros2, NodeOptions},
    rustdds::{self, policy},
    turtlesim::msg::Pose,
};
use eyre::Context;

fn main() -> eyre::Result<()> {
    let mut ros_node = init_ros_node()?;
    let turtle_vel_publisher = create_vel_publisher(&mut ros_node)?;
    let turtle_pose_reader = create_pose_reader(&mut ros_node)?;

    let output = DataId::from("pose".to_owned());

    let (mut node, dora_events) = DoraNode::init_from_env()?;

    let merged = dora_events.merge_external(Box::pin(turtle_pose_reader.async_stream()));
    let mut events = futures::executor::block_on_stream(merged);

    for i in 0..500 {
        let event = match events.next() {
            Some(input) => input,
            None => break,
        };

        match event {
            MergedEvent::Dora(event) => match event {
                Event::Input {
                    id,
                    metadata: _,
                    data: _,
                } => match id.as_str() {
                    "tick" => {
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
                        println!("tick {i}, sending {direction:?}");
                        turtle_vel_publisher.publish(direction).unwrap();
                    }
                    other => eprintln!("Ignoring unexpected input `{other}`"),
                },
                Event::Stop => println!("Received manual stop"),
                other => eprintln!("Received unexpected input: {other:?}"),
            },
            MergedEvent::External(pose) => {
                println!("received pose event: {pose:?}");
                if let Ok((pose, _)) = pose {
                    let serialized = serde_json::to_string(&pose)?;
                    node.send_output_bytes(
                        output.clone(),
                        Default::default(),
                        serialized.len(),
                        serialized.as_bytes(),
                    )?;
                }
            }
        }
    }

    Ok(())
}

fn init_ros_node() -> eyre::Result<ros2_client::Node> {
    let ros_context = ros2_client::Context::new().unwrap();

    ros_context
        .new_node(
            "turtle_teleop", // name
            "/ros2_demo",    // namespace
            NodeOptions::new().enable_rosout(true),
        )
        .context("failed to create ros2 node")
}

fn create_vel_publisher(
    ros_node: &mut ros2_client::Node,
) -> eyre::Result<ros2_client::Publisher<Twist>> {
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
        .context("failed to create topic")?;

    // The point here is to publish Twist for the turtle
    let turtle_cmd_vel_writer = ros_node
        .create_publisher::<Twist>(&turtle_cmd_vel_topic, None)
        .context("failed to create publisher")?;
    Ok(turtle_cmd_vel_writer)
}

fn create_pose_reader(
    ros_node: &mut ros2_client::Node,
) -> eyre::Result<ros2_client::Subscription<Pose>> {
    let turtle_pose_topic = ros_node
        .create_topic(
            "/turtle1/pose",
            String::from("turtlesim::msg::dds_::Pose_"),
            &Default::default(),
        )
        .context("failed to create topic")?;
    let turtle_pose_reader = ros_node
        .create_subscription::<Pose>(&turtle_pose_topic, None)
        .context("failed to create subscription")?;
    Ok(turtle_pose_reader)
}
