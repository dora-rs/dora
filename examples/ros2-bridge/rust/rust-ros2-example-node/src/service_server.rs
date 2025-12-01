use dora_node_api::{
    self, DoraNode, Event,
    merged::{MergeExternal, MergedEvent},
};
use dora_ros2_bridge::{
    messages::example_interfaces::service::{AddTwoInts, AddTwoIntsResponse},
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

    let server = create_service_server(&mut ros_node)?; // should be after the spiner started
    let (_node, dora_events) = DoraNode::init_from_env()?;
    let merged_events = dora_events.merge_external(server.receive_request_stream());

    let mut events = futures::executor::block_on_stream(merged_events);

    let mut counter = 0;
    loop {
        let event = match events.next() {
            Some(input) => input,
            None => break,
        };

        match event {
            MergedEvent::Dora(Event::Input {
                id,
                metadata: _,
                data: _,
            }) => match id.as_str() {
                "tick" => {}
                other => eprintln!("Ignoring unexpected input `{other}`"),
            },
            MergedEvent::Dora(Event::Stop(_)) => {
                println!("Received stop");
                break;
            }
            MergedEvent::External(Ok((req_id, req))) => {
                println!("request: {} + {}", req.a, req.b);
                let resp = AddTwoIntsResponse { sum: req.a + req.b };
                futures::executor::block_on(server.async_send_response(req_id, resp))?;
                counter += 1;
                if counter > 3 {
                    // only 3 times, because the service discovery is slow.
                    break;
                }
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

fn create_service_server(
    ros_node: &mut ros2_client::Node,
) -> eyre::Result<ros2_client::Server<AddTwoInts>> {
    // create an example service client
    let service_qos = {
        rustdds::QosPolicyBuilder::new()
            .reliability(policy::Reliability::Reliable {
                max_blocking_time: rustdds::Duration::from_millis(100),
            })
            .history(policy::History::KeepLast { depth: 1 })
            .build()
    };
    let add_server = ros_node
        .create_server::<AddTwoInts>(
            ros2_client::ServiceMapping::Enhanced,
            &ros2_client::Name::new("/", "add_two_ints").unwrap(),
            &ros2_client::ServiceTypeName::new("example_interfaces", "AddTwoInts"),
            service_qos.clone(),
            service_qos.clone(),
        )
        .map_err(|e| eyre::eyre!("failed to create service client: {e:?}"))?;

    Ok(add_server)
}
