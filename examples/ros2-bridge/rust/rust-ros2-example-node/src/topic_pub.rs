use dora_node_api::{self, DoraNode, Event};
use dora_ros2_bridge::{
    messages::std_msgs::msg::String as Ros2String,
    ros2_client::{self, NodeOptions, ros2},
    rustdds::{self, policy},
};
use eyre::{Context, eyre};
use futures::task::SpawnExt;

fn main() -> eyre::Result<()> {
    let mut ros_node = init_ros_node()?;
    let publisher = create_publisher(&mut ros_node)?;

    // spawn a background spinner task that is handles service discovery (and other things)
    let pool = futures::executor::ThreadPool::new()?;
    let spinner = ros_node
        .spinner()
        .map_err(|e| eyre::eyre!("failed to create spinner: {e:?}"))?;
    pool.spawn(async {
        if let Err(err) = spinner.spin().await {
            eprintln!("ros2 spinner failed: {err:?}");
        }
    })
    .context("failed to spawn ros2 spinner")?;

    let (_node, dora_events) = DoraNode::init_from_env()?;

    let mut events = futures::executor::block_on_stream(dora_events);

    for i in 0..20 {
        let event = match events.next() {
            Some(input) => input,
            None => break,
        };

        match event {
            Event::Input {
                id,
                metadata: _,
                data: _,
            } => match id.as_str() {
                "tick" => {
                    let msg = Ros2String {
                        data: format!("The {i} hello from Dora"),
                    };
                    println!("tick {i}, sending {msg:?}");
                    publisher.publish(msg).unwrap();
                }
                other => eprintln!("Ignoring unexpected input `{other}`"),
            },
            Event::Stop(_) => {
                println!("Received stop");
                break;
            }
            other => eprintln!("Received unexpected input: {other:?}"),
        }
    }

    Ok(())
}

fn init_ros_node() -> eyre::Result<ros2_client::Node> {
    let ros_context = ros2_client::Context::new().unwrap();

    ros_context
        .new_node(
            ros2_client::NodeName::new("/", "ros2_dora_pub")
                .map_err(|e| eyre!("failed to create ROS2 node name: {e}"))?,
            NodeOptions::new().enable_rosout(true),
        )
        .map_err(|e| eyre::eyre!("failed to create ros2 node: {e:?}"))
}

fn create_publisher(
    ros_node: &mut ros2_client::Node,
) -> eyre::Result<ros2_client::Publisher<Ros2String>> {
    let topic_qos: rustdds::QosPolicies = {
        rustdds::QosPolicyBuilder::new()
            .durability(policy::Durability::Volatile)
            .liveliness(policy::Liveliness::Automatic {
                lease_duration: ros2::Duration::INFINITE,
            })
            .reliability(policy::Reliability::Reliable {
                max_blocking_time: ros2::Duration::from_millis(100),
            })
            .history(policy::History::KeepLast { depth: 1 })
            .build()
    };

    let topic = ros_node
        .create_topic(
            &ros2_client::Name::new("/", "topic")
                .map_err(|e| eyre!("failed to create ROS2 name: {e}"))?,
            ros2_client::MessageTypeName::new("std_msgs", "String"),
            &topic_qos,
        )
        .context("failed to create topic")?;

    // The point here is to publish Twist for the turtle
    let publisher = ros_node
        .create_publisher::<Ros2String>(&topic, None)
        .context("failed to create publisher")?;
    Ok(publisher)
}
