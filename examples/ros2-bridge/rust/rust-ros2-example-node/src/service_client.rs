use std::time::Duration;

use dora_node_api::{self, DoraNode, Event};
use dora_ros2_bridge::{
    messages::example_interfaces::service::{AddTwoInts, AddTwoIntsRequest},
    ros2_client::{self, NodeOptions},
    rustdds::{self, policy},
};
use eyre::{Context, eyre};
use futures::task::SpawnExt;

fn main() -> eyre::Result<()> {
    let mut ros_node = init_ros_node()?;

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

    let client = create_service_client(&mut ros_node)?; // should be after the spiner started
    let (_node, mut dora_events) = DoraNode::init_from_env()?;

    for i in 0..20 {
        let event = match dora_events.recv() {
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
                    let a = rand::random();
                    let b = rand::random();
                    let request = AddTwoIntsRequest { a, b };

                    println!("tick {i}, sending {request:?}");
                    let req_id = client.send_request(request)?;

                    futures::executor::block_on(async {
                        let response = client
                            .async_receive_response(req_id)
                            .await
                            .expect("failed to receive response");
                        println!("tick {i}, received {response:?}");
                    });
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
            ros2_client::NodeName::new("/", "ros2_dora_service_client")
                .map_err(|e| eyre!("failed to create ROS2 node name: {e}"))?,
            NodeOptions::new().enable_rosout(true),
        )
        .map_err(|e| eyre::eyre!("failed to create ros2 node: {e:?}"))
}

fn create_service_client(
    ros_node: &mut ros2_client::Node,
) -> eyre::Result<ros2_client::Client<AddTwoInts>> {
    // create an example service client
    let service_qos = {
        rustdds::QosPolicyBuilder::new()
            .reliability(policy::Reliability::Reliable {
                max_blocking_time: rustdds::Duration::from_millis(100),
            })
            .history(policy::History::KeepLast { depth: 1 })
            .build()
    };
    let add_client = ros_node
        .create_client::<AddTwoInts>(
            ros2_client::ServiceMapping::Enhanced,
            &ros2_client::Name::new("/", "add_two_ints").unwrap(),
            &ros2_client::ServiceTypeName::new("example_interfaces", "AddTwoInts"),
            service_qos.clone(),
            service_qos.clone(),
        )
        .map_err(|e| eyre::eyre!("failed to create service client: {e:?}"))?;

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
        eyre::bail!("add_two_ints service not available");
    };
    futures::executor::block_on(service_ready)?;
    Ok(add_client)
}
