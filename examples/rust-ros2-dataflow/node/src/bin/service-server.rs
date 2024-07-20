use std::time::Duration;

use dora_node_api::{
    self,
    dora_core::config::DataId,
    merged::{MergeExternal, MergedEvent},
    DoraNode, Event,
};
use dora_ros2_bridge::{
    messages::{
        example_interfaces::service::{AddTwoInts, AddTwoIntsRequest, AddTwoIntsResponse},
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

    let service_qos = {
        rustdds::QosPolicyBuilder::new()
            .reliability(policy::Reliability::Reliable {
                max_blocking_time: rustdds::Duration::from_millis(100),
            })
            .history(policy::History::KeepLast { depth: 10 })
            .build()
    };
    let server = ros_node
        .create_server::<AddTwoInts>(
            ros2_client::ServiceMapping::Enhanced,
            &ros2_client::Name::new("/", "add_two_ints_custom").unwrap(),
            &ros2_client::ServiceTypeName::new("example_interfaces", "AddTwoInts"),
            service_qos.clone(),
            service_qos.clone(),
        )
        .context("failed to create service server")?;

    let (node, dora_events) = DoraNode::init_from_env()?;

    let merged = dora_events.merge_external(Box::pin(server.receive_request_stream()));
    let mut events = futures::executor::block_on_stream(merged);

    loop {
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
                    "exit" => {
                        println!("received exit signal");
                        break;
                    }
                    other => eprintln!("Ignoring unexpected input `{other}`"),
                },
                Event::Stop => println!("Received manual stop"),
                other => eprintln!("Received unexpected input: {other:?}"),
            },
            MergedEvent::External(request) => match request {
                Ok((id, request)) => {
                    let response = AddTwoIntsResponse {
                        sum: request.a.wrapping_add(request.b),
                    };
                    println!("replying to incoming request {id:?} {request:?} with response {response:?}");
                    server
                        .send_response(id, response)
                        .context("failed to send response")?;
                }
                Err(err) => eprintln!("error while receiving incoming request: {err:?}"),
            },
        }
    }

    Ok(())
}

fn init_ros_node() -> eyre::Result<ros2_client::Node> {
    let ros_context = ros2_client::Context::new().unwrap();

    ros_context
        .new_node(
            ros2_client::NodeName::new("/ros2_demo", "service_server_example")
                .map_err(|e| eyre!("failed to create ROS2 node name: {e}"))?,
            NodeOptions::new().enable_rosout(true),
        )
        .context("failed to create ros2 node")
}
