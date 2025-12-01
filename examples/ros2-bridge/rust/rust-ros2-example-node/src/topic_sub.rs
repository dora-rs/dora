use dora_node_api::{
    self, DoraNode, Event,
    merged::{MergeExternal, MergedEvent},
};
use dora_ros2_bridge::{
    messages::std_msgs::msg::String as Ros2String,
    ros2_client::{self, NodeOptions},
};
use eyre::{Context, eyre};
use futures::task::SpawnExt;

fn main() -> eyre::Result<()> {
    let mut ros_node = init_ros_node()?;
    let subscriber = create_subscriber(&mut ros_node)?;

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

    let merged_events = dora_events.merge_external(Box::pin(subscriber.async_stream()));
    let mut events = futures::executor::block_on_stream(merged_events);

    let mut count = 0usize;
    while let Some(event) = events.next() {
        match event {
            MergedEvent::Dora(event) => match event {
                Event::Input {
                    id,
                    metadata: _,
                    data: _,
                } => match id.as_str() {
                    "tick" => {}
                    other => eprintln!("Ignoring unexpected input `{other}`"),
                },
                Event::Stop(_) => {
                    println!("Received stop");
                    break;
                }
                other => eprintln!("Received unexpected input: {other:?}"),
            },
            MergedEvent::External(recv) => {
                println!("received external event: {:?}", recv);
                count += 1;
                if count > 20 {
                    break;
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
            ros2_client::NodeName::new("/", "ros2_dora_sub")
                .map_err(|e| eyre!("failed to create ROS2 node name: {e}"))?,
            NodeOptions::new().enable_rosout(true),
        )
        .map_err(|e| eyre::eyre!("failed to create ros2 node: {e:?}"))
}

fn create_subscriber(
    ros_node: &mut ros2_client::Node,
) -> eyre::Result<ros2_client::Subscription<Ros2String>> {
    let topic = ros_node
        .create_topic(
            &ros2_client::Name::new("/", "topic")
                .map_err(|e| eyre!("failed to create ROS2 name: {e}"))?,
            ros2_client::MessageTypeName::new("std_msgs", "String"),
            &Default::default(),
        )
        .context("failed to create topic")?;
    let subscriber = ros_node
        .create_subscription::<Ros2String>(&topic, None)
        .context("failed to create subscription")?;
    Ok(subscriber)
}
