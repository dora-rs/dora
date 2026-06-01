use dora_node_api::{
    self, DoraNode, Event, flume,
    merged::{MergeExternal, MergedEvent},
};
use dora_ros2_bridge::{
    messages::example_interfaces::action::{Fibonacci, FibonacciFeedback, FibonacciResult},
    ros2_client::{
        self, NodeOptions,
        action::{ActionServerQosPolicies, AsyncActionServer, GoalEndStatus},
    },
    rustdds::{self, policy},
};
use eyre::{Context, eyre};
use futures::task::SpawnExt;
use std::{sync::Arc, time::Duration};

fn main() -> eyre::Result<()> {
    let mut ros_node = init_ros_node()?;

    // spawn a background spinner task that handles service discovery (and other things)
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

    let server = Arc::new(create_action_server(&mut ros_node)?); // after the spinner started

    // The action FSM (receive -> accept -> execute -> feedback -> result) runs in
    // a background task; it notifies the dora loop after each goal is served.
    let (served_tx, served_rx) = flume::unbounded::<i32>();
    let server_task = server.clone();
    pool.spawn(async move {
        loop {
            match serve_one_goal(&server_task).await {
                Ok(order) => {
                    let _ = served_tx.send(order);
                }
                Err(err) => {
                    eprintln!("action server: failed to serve goal: {err:?}");
                    // Back off so a persistently-failing receive can't busy-spin
                    // (and flood the log) for the lifetime of the node.
                    futures_timer::Delay::new(Duration::from_millis(200)).await;
                }
            }
        }
    })
    .context("failed to spawn action server task")?;

    let (_node, dora_events) = DoraNode::init_from_env()?;
    let merged_events = dora_events.merge_external(Box::pin(served_rx.into_stream()));
    let mut events = futures::executor::block_on_stream(merged_events);

    let mut served = 0;
    loop {
        let event = match events.next() {
            Some(input) => input,
            None => break,
        };

        match event {
            MergedEvent::Dora(Event::Input { id, .. }) => match id.as_str() {
                "tick" => {}
                other => eprintln!("Ignoring unexpected input `{other}`"),
            },
            MergedEvent::Dora(Event::Stop(_)) => {
                println!("Received stop");
                break;
            }
            MergedEvent::Dora(other) => eprintln!("Received unexpected event: {other:?}"),
            MergedEvent::External(order) => {
                println!("served fibonacci goal (order {order})");
                served += 1;
                // Stop after a few goals; ROS2 discovery is slow, so a real
                // client may only connect once during the smoke window.
                if served >= 1 {
                    break;
                }
            }
        }
    }

    Ok(())
}

/// Handle exactly one goal: accept it, compute the Fibonacci sequence while
/// publishing feedback, then send the final result. Returns the goal's `order`.
async fn serve_one_goal(server: &AsyncActionServer<Fibonacci>) -> eyre::Result<i32> {
    let new_goal = server
        .receive_new_goal()
        .await
        .map_err(|e| eyre!("failed to receive goal: {e:?}"))?;
    let order = server
        .get_new_goal(new_goal.clone())
        .ok_or_else(|| eyre!("received goal contained no data"))?
        .order;

    eprintln!("action server: received goal (order {order}), accepting");
    let accepted = server
        .accept_goal(new_goal)
        .await
        .map_err(|e| eyre!("failed to accept goal: {e:?}"))?;
    let executing = server
        .start_executing_goal(accepted)
        .await
        .map_err(|e| eyre!("failed to start executing goal: {e:?}"))?;
    eprintln!("action server: executing goal (order {order})");

    // Fibonacci sequence, mirroring rclcpp's minimal_action_server: seed [0, 1]
    // then append `order` more terms, publishing the running sequence as feedback.
    let mut sequence: Vec<i32> = vec![0, 1];
    for i in 1..order.max(1) {
        // `saturating_add`: Fibonacci overflows i32 around order 47; saturate
        // instead of panicking (debug) / wrapping (release) on a large goal.
        let next = sequence[(i - 1) as usize].saturating_add(sequence[i as usize]);
        sequence.push(next);
        server
            .publish_feedback(
                executing.clone(),
                FibonacciFeedback {
                    sequence: sequence.clone(),
                },
            )
            .await
            .map_err(|e| eyre!("failed to publish feedback: {e:?}"))?;
    }
    eprintln!("action server: published feedback, sending result (order {order})");

    server
        .send_result_response(
            executing,
            GoalEndStatus::Succeeded,
            FibonacciResult { sequence },
        )
        .await
        .map_err(|e| eyre!("failed to send result: {e:?}"))?;
    Ok(order)
}

fn init_ros_node() -> eyre::Result<ros2_client::Node> {
    let ros_context = ros2_client::Context::new().unwrap();

    ros_context
        .new_node(
            ros2_client::NodeName::new("/", "ros2_dora_action_server")
                .map_err(|e| eyre!("failed to create ROS2 node name: {e}"))?,
            NodeOptions::new().enable_rosout(true),
        )
        .map_err(|e| eyre::eyre!("failed to create ros2 node: {e:?}"))
}

fn create_action_server(
    ros_node: &mut ros2_client::Node,
) -> eyre::Result<AsyncActionServer<Fibonacci>> {
    let action_qos = {
        let service_qos = rustdds::QosPolicyBuilder::new()
            .reliability(policy::Reliability::Reliable {
                max_blocking_time: rustdds::Duration::from_millis(100),
            })
            .history(policy::History::KeepLast { depth: 1 })
            .build();
        ActionServerQosPolicies {
            goal_service: service_qos.clone(),
            result_service: service_qos.clone(),
            cancel_service: service_qos.clone(),
            feedback_publisher: service_qos.clone(),
            status_publisher: service_qos,
        }
    };
    let server = ros_node
        .create_action_server::<Fibonacci>(
            ros2_client::ServiceMapping::Enhanced,
            &ros2_client::Name::new("/", "fibonacci").unwrap(),
            &ros2_client::ActionTypeName::new("example_interfaces", "Fibonacci"),
            action_qos,
        )
        .map_err(|e| eyre::eyre!("failed to create action server: {e:?}"))?;
    Ok(AsyncActionServer::new(server))
}
