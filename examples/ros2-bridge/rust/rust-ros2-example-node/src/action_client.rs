use futures::StreamExt;
use std::time::Duration;

use dora_node_api::{
    self, DoraNode, Event,
    merged::{MergeExternal, MergedEvent},
};
use dora_ros2_bridge::{
    messages::{
        self,
        example_interfaces::action::{Fibonacci, FibonacciGoal},
    },
    ros2_client::{
        self, ActionTypes, NodeOptions,
        action::{GoalId, GoalStatusEnum},
    },
    rustdds::{self, policy},
};
use eyre::{Context, eyre};
use futures::task::SpawnExt;

#[derive(Debug)]
enum ActionEvent<T>
where
    T: ActionTypes,
{
    Result {
        id: GoalId,
        result: Result<(GoalStatusEnum, T::ResultType), ros2_client::service::CallServiceError<()>>,
    },
    Feedback {
        id: GoalId,
        feedback: rustdds::dds::result::ReadResult<T::FeedbackType>,
    },
    Status {
        id: GoalId,
        status: rustdds::dds::result::ReadResult<messages::action_msgs::msg::GoalStatus>,
    },
}

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

    let client = create_action_client(&mut ros_node)?; // should be after the spiner started
    let (client_stream, mut client_stream_handle) =
        futures_concurrency_dynamic::dynamic_merge_with_handle::<ActionEvent<Fibonacci>>();
    let (_node, dora_events) = DoraNode::init_from_env()?;
    let merged_events = dora_events.merge_external(Box::pin(client_stream));
    let mut events = futures::executor::block_on_stream(merged_events);

    let mut sent = false;
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
                    "tick" => {
                        if !sent {
                            let goal = FibonacciGoal { order: 10 };
                            futures::executor::block_on(async {
                                if let Ok((goal_id, goal_resp)) = client.async_send_goal(goal).await
                                {
                                    println!("goal_resp: {:?}", goal_resp);
                                    if goal_resp.accepted {
                                        let feedback_stream =
                                            client.feedback_stream(goal_id).map(move |feedback| {
                                                ActionEvent::Feedback {
                                                    id: goal_id,
                                                    feedback,
                                                }
                                            });
                                        let status_stream =
                                            client.status_stream(goal_id).map(move |status| {
                                                ActionEvent::Status {
                                                    id: goal_id,
                                                    status: status.map(|status| status.into()),
                                                }
                                            });

                                        client_stream_handle.push(feedback_stream);
                                        client_stream_handle.push(result_stream(&client, goal_id));
                                        client_stream_handle.push(status_stream);
                                        sent = true;
                                    }
                                }
                            });
                        }
                    }
                    other => eprintln!("Ignoring unexpected input `{other}`"),
                },
                Event::Stop(_) => println!("Received stop"),
                other => eprintln!("Received unexpected input: {other:?}"),
            },
            MergedEvent::External(action_event) => match action_event {
                ActionEvent::Result { id, result } => {
                    if let Ok((status, result)) = result {
                        if status == GoalStatusEnum::Unknown {
                            client_stream_handle.push(result_stream(&client, id));
                            // there are tons of unknown status :(
                        } else {
                            println!("status: {:?}", result);
                            break;
                        }
                    }
                }
                other => {
                    println!("{:?}", other);
                }
            },
        }
    }

    Ok(())
}

fn result_stream<T>(
    client: &ros2_client::action::ActionClient<T>,
    goal_id: GoalId,
) -> impl futures::stream::Stream<Item = ActionEvent<T>>
where
    T: ActionTypes,
    <T as ActionTypes>::ResultType: 'static,
{
    let fut = client.async_request_result(goal_id);
    futures::stream::once(async move {
        ActionEvent::Result {
            id: goal_id,
            result: fut.await,
        }
    })
    .fuse()
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
            let ready = fib_client.goal_client().wait_for_service(&ros_node);
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
