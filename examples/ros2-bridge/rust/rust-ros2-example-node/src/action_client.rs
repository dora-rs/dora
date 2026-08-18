use std::time::{Duration, Instant};

use dora_node_api::{DoraNode, Event};
use dora_ros2_bridge::{
    messages::example_interfaces::action::{Fibonacci, FibonacciGoal},
    ros2_client::{
        self, ActionTypes, NodeOptions,
        action::{GoalId, GoalStatusEnum},
        service::RmwRequestId,
    },
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

    let mut client = create_action_client(&mut ros_node)?; // should be after the spiner started
    let (_node, dora_events) = DoraNode::init_from_env()?;
    let mut events = futures::executor::block_on_stream(dora_events);

    let deadline = Instant::now() + Duration::from_secs(60);
    let mut sent_goal: Option<GoalId> = None;
    let mut pending_results: Vec<RmwRequestId> = Vec::new();
    loop {
        if Instant::now() >= deadline {
            eyre::bail!("timed out waiting for a terminal Fibonacci action result");
        }
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
                    if sent_goal.is_none() {
                        let goal = FibonacciGoal { order: 10 };
                        let (goal_id, goal_resp) = futures::executor::block_on(
                            client.async_send_goal(goal),
                        )
                        .map_err(|err| eyre::eyre!("failed to send Fibonacci goal: {err:?}"))?;
                        println!("goal_resp: {:?}", goal_resp);
                        if !goal_resp.accepted {
                            eyre::bail!("Fibonacci goal was rejected by the action server");
                        }
                        sent_goal = Some(goal_id);
                    }

                    if let Some(goal_id) = sent_goal
                        && pending_results.is_empty()
                    {
                        pending_results.push(request_result(&client, goal_id)?);
                    }

                    match receive_result(&mut client, &mut pending_results)? {
                        Some((GoalStatusEnum::Unknown, _)) => {}
                        Some((GoalStatusEnum::Succeeded, result)) => {
                            println!("status: {:?}", result);
                            break;
                        }
                        Some((GoalStatusEnum::Canceled, _)) => {
                            eyre::bail!("Fibonacci goal was canceled");
                        }
                        Some((GoalStatusEnum::Aborted, _)) => {
                            eyre::bail!("Fibonacci goal was aborted");
                        }
                        Some((status, _)) => {
                            println!("Fibonacci goal still running with status: {status:?}");
                        }
                        None => {}
                    }
                }
                other => eprintln!("Ignoring unexpected input `{other}`"),
            },
            Event::Stop(_) => println!("Received stop"),
            other => eprintln!("Received unexpected input: {other:?}"),
        }
    }

    Ok(())
}

fn request_result<T>(
    client: &ros2_client::action::ActionClient<T>,
    goal_id: GoalId,
) -> eyre::Result<RmwRequestId>
where
    T: ActionTypes,
    <T as ActionTypes>::ResultType: 'static,
{
    client
        .request_result(goal_id)
        .map_err(|err| eyre::eyre!("failed to request Fibonacci result: {err:?}"))
}

fn receive_result<T>(
    client: &mut ros2_client::action::ActionClient<T>,
    pending_results: &mut Vec<RmwRequestId>,
) -> eyre::Result<Option<(GoalStatusEnum, T::ResultType)>>
where
    T: ActionTypes,
    <T as ActionTypes>::ResultType: 'static,
{
    loop {
        match client.result_client().receive_response() {
            Ok(Some((incoming_req_id, response))) => {
                if let Some(pos) = pending_results.iter().position(|id| *id == incoming_req_id) {
                    pending_results.remove(pos);
                    break Ok(Some((response.status, response.result)));
                }
            }
            Ok(None) => break Ok(None),
            Err(err) => eyre::bail!("failed to receive Fibonacci result: {err:?}"),
        }
    }
}

fn init_ros_node() -> eyre::Result<ros2_client::Node> {
    let ros_context = ros2_client::Context::new().unwrap();

    ros_context
        .new_node(
            ros2_client::NodeName::new("/", "ros2_dora_action_client")
                .map_err(|e| eyre!("failed to create ROS2 node name: {e}"))?,
            NodeOptions::new().enable_rosout(true),
        )
        .map_err(|e| eyre::eyre!("failed to create ros2 node: {e:?}"))
}

fn create_action_client(
    ros_node: &mut ros2_client::Node,
) -> eyre::Result<ros2_client::action::ActionClient<Fibonacci>> {
    // create an example service client
    let action_qos = {
        let service_qos = rustdds::QosPolicyBuilder::new()
            .reliability(policy::Reliability::Reliable {
                max_blocking_time: rustdds::Duration::from_millis(100),
            })
            .history(policy::History::KeepLast { depth: 1 })
            .build();
        ros2_client::action::ActionClientQosPolicies {
            goal_service: service_qos.clone(),
            result_service: service_qos.clone(),
            cancel_service: service_qos.clone(),
            feedback_subscription: service_qos.clone(),
            status_subscription: service_qos,
        }
    };
    let mut fib_client = ros_node
        .create_action_client::<Fibonacci>(
            ros2_client::ServiceMapping::Enhanced,
            &ros2_client::Name::new("/", "fibonacci").unwrap(),
            &ros2_client::ActionTypeName::new("example_interfaces", "Fibonacci"),
            action_qos,
        )
        .map_err(|e| eyre::eyre!("failed to create service client: {e:?}"))?;

    println!("wait for fibonacci action server");
    let server_ready = async {
        for _ in 0..10 {
            let ready = fib_client.goal_client().wait_for_service(ros_node);
            futures::pin_mut!(ready);
            let timeout = futures_timer::Delay::new(Duration::from_secs(2));
            match futures::future::select(ready, timeout).await {
                futures::future::Either::Left(((), _)) => {
                    println!("action server is ready");
                    return Ok(());
                }
                futures::future::Either::Right(_) => {
                    println!("timeout while waiting for action server, retrying");
                }
            }
        }
        eyre::bail!("action server not available");
    };
    futures::executor::block_on(server_ready)?;
    Ok(fib_client)
}
