use std::time::Duration;

use dora_node_api::{
    self,
    dora_core::config::DataId,
    merged::{MergeExternal, MergedEvent},
    DoraNode, Event,
};
use dora_ros2_bridge::{
    messages::{
        example_interfaces::service::{AddTwoInts, AddTwoIntsRequest},
        geometry_msgs::msg::{Twist, Vector3},
        turtlesim::msg::Pose,
    },
    ros2_client::{self, ros2, NodeOptions},
    rustdds::{self, policy},
};
use eyre::{eyre, Context};
use futures::task::SpawnExt;

fn main() -> eyre::Result<()> {
    let mut ros_node = init_ros_node()?;
    let turtle_vel_publisher = create_vel_publisher(&mut ros_node)?;
    let turtle_pose_reader = create_pose_reader(&mut ros_node)?;

    // spawn a background spinner task that is handles service discovery (and other things)
    let pool = futures::executor::ThreadPool::new()?;
    let spinner = ros_node.spinner();
    pool.spawn(async {
        if let Err(err) = spinner.spin().await {
            eprintln!("ros2 spinner failed: {err:?}");
        }
    })
    .context("failed to spawn ros2 spinner")?;

    // create an example service client
    let service_qos = {
        rustdds::QosPolicyBuilder::new()
            .reliability(policy::Reliability::Reliable {
                max_blocking_time: rustdds::Duration::from_millis(100),
            })
            .history(policy::History::KeepLast { depth: 1 })
            .build()
    };
    let add_client = ros_node.create_client::<AddTwoInts>(
        ros2_client::ServiceMapping::Enhanced,
        &ros2_client::Name::new("/", "add_two_ints").unwrap(),
        &ros2_client::ServiceTypeName::new("example_interfaces", "AddTwoInts"),
        service_qos.clone(),
        service_qos.clone(),
    )?;

    // wait until the service server is ready
    println!("wait for add_two_ints service");
    let service_ready = async {
        for _ in 0..10 {
            let ready = add_client.wait_for_service(&ros_node);
            futures::pin_mut!(ready);
            let timeout = futures_timer::Delay::new(Duration::from_secs(2));
            match futures::future::select(ready, timeout).await {
                futures::future::Either::Left(((), _)) => {
                    println!("add_two_ints service is ready");
                    return Ok(());
                }
                futures::future::Either::Right(_) => {
                    println!("timeout while waiting for add_two_ints service, retrying");
                }
            }
        }
        eyre::bail!("add_to_ints service not available");
    };
    futures::executor::block_on(service_ready)?;

    let output = DataId::from("pose".to_owned());

    let (mut node, dora_events) = DoraNode::init_from_env()?;

    let merged = dora_events.merge_external(Box::pin(turtle_pose_reader.async_stream()));
    let mut events = futures::executor::block_on_stream(merged);

    for i in 0..1000 {
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
                    "service_timer" => {
                        let a = rand::random();
                        let b = rand::random();
                        let service_result = add_two_ints_request(&add_client, a, b);
                        let sum = futures::executor::block_on(service_result)
                            .context("failed to send service request")?;
                        if sum != a.wrapping_add(b) {
                            eyre::bail!("unexpected addition result: expected {}, got {sum}", a + b)
                        }
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

async fn add_two_ints_request(
    add_client: &ros2_client::Client<AddTwoInts>,
    a: i64,
    b: i64,
) -> eyre::Result<i64> {
    let request = AddTwoIntsRequest { a, b };
    println!("sending add request {request:?}");
    let request_id = add_client.async_send_request(request.clone()).await?;
    println!("{request_id:?}");

    let response = add_client.async_receive_response(request_id);
    futures::pin_mut!(response);
    let timeout = futures_timer::Delay::new(Duration::from_secs(5));
    match futures::future::select(response, timeout).await {
        futures::future::Either::Left((Ok(response), _)) => {
            println!("received response: {response:?}");
            Ok(response.sum)
        }
        futures::future::Either::Left((Err(err), _)) => eyre::bail!(err),
        futures::future::Either::Right(_) => {
            eyre::bail!("timeout while waiting for response");
        }
    }
}

fn init_ros_node() -> eyre::Result<ros2_client::Node> {
    let ros_context = ros2_client::Context::new().unwrap();

    ros_context
        .new_node(
            ros2_client::NodeName::new("/ros2_demo", "turtle_teleop")
                .map_err(|e| eyre!("failed to create ROS2 node name: {e}"))?,
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
                lease_duration: ros2::Duration::INFINITE,
            })
            .reliability(policy::Reliability::Reliable {
                max_blocking_time: ros2::Duration::from_millis(100),
            })
            .history(policy::History::KeepLast { depth: 1 })
            .build()
    };

    let turtle_cmd_vel_topic = ros_node
        .create_topic(
            &ros2_client::Name::new("/turtle1", "cmd_vel")
                .map_err(|e| eyre!("failed to create ROS2 name: {e}"))?,
            ros2_client::MessageTypeName::new("geometry_msgs", "Twist"),
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
            &ros2_client::Name::new("/turtle1", "pose")
                .map_err(|e| eyre!("failed to create ROS2 name: {e}"))?,
            ros2_client::MessageTypeName::new("turtlesim", "Pose"),
            &Default::default(),
        )
        .context("failed to create topic")?;
    let turtle_pose_reader = ros_node
        .create_subscription::<Pose>(&turtle_pose_topic, None)
        .context("failed to create subscription")?;
    Ok(turtle_pose_reader)
}
