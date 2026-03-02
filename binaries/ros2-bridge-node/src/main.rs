use std::{
    borrow::Cow,
    collections::{BTreeMap, HashMap},
    path::Path,
    sync::{
        Arc,
        atomic::{AtomicUsize, Ordering},
    },
    time::Duration,
};

use adora_message::{
    descriptor::{Ros2BridgeConfig, Ros2Direction, Ros2QosConfig, Ros2Role, Ros2TopicConfig},
    metadata::{Parameter, get_string_param},
};
use adora_node_api::{
    AdoraNode, Event,
    merged::{MergeExternal, MergedEvent},
};
use adora_ros2_bridge::{ros2_client, rustdds};
use adora_ros2_bridge_arrow::{
    BridgeActionType, BridgeMessage, BridgeServiceType, TypeInfo, TypeInfoGuard, TypedValue,
    deserialize::StructDeserializer,
};
use adora_ros2_bridge_msg_gen::types::Message;
use arrow::array::{ArrayData, StructArray};
use eyre::{Context, ContextCompat, eyre};
use futures::{StreamExt, task::SpawnExt};

/// Maximum number of concurrent in-flight action goals.
const MAX_CONCURRENT_GOALS: usize = 8;

/// Maximum pending service requests before dropping new ones.
const MAX_PENDING_REQUESTS: usize = 64;

use adora_message::metadata::{
    GOAL_ID, GOAL_STATUS, GOAL_STATUS_ABORTED, GOAL_STATUS_CANCELED, GOAL_STATUS_SUCCEEDED,
    REQUEST_ID,
};

fn main() -> eyre::Result<()> {
    tracing_subscriber::fmt::init();

    let config_json = std::env::var("ADORA_ROS2_BRIDGE_CONFIG")
        .context("missing ADORA_ROS2_BRIDGE_CONFIG env var")?;
    let config: Ros2BridgeConfig =
        serde_json::from_str(&config_json).context("failed to parse ROS2 bridge config")?;

    let messages = load_messages()?;

    // Dispatch based on bridge mode
    if config.service.is_some() {
        return run_service_mode(config, messages);
    }
    if config.action.is_some() {
        return run_action_mode(config, messages);
    }

    // Topic mode (existing behavior)
    run_topic_mode(config, messages)
}

// ---------------------------------------------------------------------------
// Topic mode (existing)
// ---------------------------------------------------------------------------

fn run_topic_mode(
    config: Ros2BridgeConfig,
    messages: Arc<HashMap<String, HashMap<String, Message>>>,
) -> eyre::Result<()> {
    let topics = resolve_topics(&config)?;

    let (mut ros_node, _pool) = create_ros_node(&config)?;

    let mut subscribers: Vec<(String, SubscriptionStream)> = Vec::new();
    let mut publishers: Vec<(
        String,
        TypeInfo<'static>,
        ros2_client::Publisher<TypedValue<'static>>,
    )> = Vec::new();

    for topic_config in &topics {
        let (package, msg_name) = parse_type_str(&topic_config.message_type)?;
        let qos = topic_config
            .qos
            .as_ref()
            .unwrap_or(&config.qos)
            .to_rustdds_qos();

        let ros_topic = ros_node
            .create_topic(
                &ros2_client::Name::new("/", topic_config.topic.trim_start_matches('/'))
                    .map_err(|e| eyre!("failed to create ROS2 topic name: {e}"))?,
                ros2_client::MessageTypeName::new(&package, &msg_name),
                &qos,
            )
            .context("failed to create ROS2 topic")?;

        let type_info = TypeInfo {
            package_name: Cow::Owned(package.clone()),
            message_name: Cow::Owned(msg_name.clone()),
            messages: messages.clone(),
        };

        match &topic_config.direction {
            Ros2Direction::Subscribe => {
                let output_id = topic_config.output.clone().unwrap_or_else(|| {
                    topic_config.topic.trim_start_matches('/').replace('/', "_")
                });
                let subscription: ros2_client::Subscription<ArrayData> = ros_node
                    .create_subscription(&ros_topic, None)
                    .context("failed to create ROS2 subscription")?;
                let deserializer = StructDeserializer::new(Cow::Owned(type_info));
                subscribers.push((
                    output_id,
                    SubscriptionStream {
                        deserializer,
                        subscription,
                    },
                ));
            }
            Ros2Direction::Publish => {
                let input_id = topic_config.input.clone().unwrap_or_else(|| {
                    topic_config.topic.trim_start_matches('/').replace('/', "_")
                });
                let publisher = ros_node
                    .create_publisher::<TypedValue<'static>>(&ros_topic, None)
                    .context("failed to create ROS2 publisher")?;
                publishers.push((input_id, type_info, publisher));
            }
        }
    }

    let (node, adora_events) = AdoraNode::init_from_env()?;

    if subscribers.is_empty() {
        run_publish_only(node, adora_events, publishers, &messages)?;
    } else if subscribers.len() == 1 {
        let (output_id, sub) = subscribers.into_iter().next().unwrap();
        run_single_subscriber(node, adora_events, output_id, sub, publishers, &messages)?;
    } else {
        run_multi_subscriber(node, adora_events, subscribers, publishers, &messages)?;
    }

    Ok(())
}

// ---------------------------------------------------------------------------
// Service mode
// ---------------------------------------------------------------------------

fn run_service_mode(
    config: Ros2BridgeConfig,
    messages: Arc<HashMap<String, HashMap<String, Message>>>,
) -> eyre::Result<()> {
    let service_name = config.service.as_ref().unwrap();
    let service_type = config
        .service_type
        .as_ref()
        .context("service_type required")?;
    let role = config.role.as_ref().context("role required for service")?;
    let (package, type_name) = parse_type_str(service_type)?;

    let req_msg_name = format!("{type_name}_Request");
    let resp_msg_name = format!("{type_name}_Response");

    let request_type_info = TypeInfo {
        package_name: Cow::Owned(package.clone()),
        message_name: Cow::Owned(req_msg_name.clone()),
        messages: messages.clone(),
    };
    let response_type_info = TypeInfo {
        package_name: Cow::Owned(package.clone()),
        message_name: Cow::Owned(resp_msg_name.clone()),
        messages: messages.clone(),
    };

    let (mut ros_node, _pool) = create_ros_node(&config)?;

    let service_qos = config.qos.to_rustdds_qos();

    match role {
        Ros2Role::Client => {
            let client = ros_node
                .create_client::<BridgeServiceType>(
                    ros2_client::ServiceMapping::Enhanced,
                    &ros2_client::Name::new("/", service_name.trim_start_matches('/'))
                        .map_err(|e| eyre!("failed to create service name: {e}"))?,
                    &ros2_client::ServiceTypeName::new(&package, &type_name),
                    service_qos.clone(),
                    service_qos,
                )
                .map_err(|e| eyre!("failed to create service client: {e:?}"))?;

            // Wait for service availability
            wait_for_service(&client, &ros_node)?;

            run_service_client(client, request_type_info, response_type_info)
        }
        Ros2Role::Server => {
            let server = ros_node
                .create_server::<BridgeServiceType>(
                    ros2_client::ServiceMapping::Enhanced,
                    &ros2_client::Name::new("/", service_name.trim_start_matches('/'))
                        .map_err(|e| eyre!("failed to create service name: {e}"))?,
                    &ros2_client::ServiceTypeName::new(&package, &type_name),
                    service_qos.clone(),
                    service_qos,
                )
                .map_err(|e| eyre!("failed to create service server: {e:?}"))?;

            run_service_server(server, request_type_info, response_type_info)
        }
    }
}

/// Timeout for service responses (30 seconds).
const SERVICE_RESPONSE_TIMEOUT: Duration = Duration::from_secs(30);

/// Timeout for action goal sends (30 seconds).
const ACTION_GOAL_TIMEOUT: Duration = Duration::from_secs(30);

/// Timeout for action result retrieval (5 minutes, actions can be long-running).
const ACTION_RESULT_TIMEOUT: Duration = Duration::from_secs(300);

fn run_service_client(
    client: ros2_client::Client<BridgeServiceType>,
    request_type_info: TypeInfo<'static>,
    response_type_info: TypeInfo<'static>,
) -> eyre::Result<()> {
    let (mut node, adora_events) = AdoraNode::init_from_env()?;

    for event in futures::executor::block_on_stream(adora_events) {
        match event {
            Event::Input {
                id: _,
                metadata: _,
                data,
            } => {
                let array_data = data.to_data();

                // Serialize request (guard clears thread-local on drop)
                let _guard = TypeInfoGuard::serialize(request_type_info.clone());
                let req_id = client
                    .send_request(BridgeMessage(Some(array_data)))
                    .map_err(|e| eyre!("failed to send service request: {e:?}"))?;

                // Receive response with timeout
                let _guard = TypeInfoGuard::deserialize(response_type_info.clone());
                let response = futures::executor::block_on(async {
                    let recv = client.async_receive_response(req_id);
                    futures::pin_mut!(recv);
                    let timeout = futures_timer::Delay::new(SERVICE_RESPONSE_TIMEOUT);
                    match futures::future::select(recv, timeout).await {
                        futures::future::Either::Left((result, _)) => {
                            result.map_err(|e| eyre!("failed to receive service response: {e:?}"))
                        }
                        futures::future::Either::Right(_) => {
                            eyre::bail!("service response timed out")
                        }
                    }
                })?;

                if let Some(resp_data) = response.0 {
                    node.send_output(
                        "response".into(),
                        Default::default(),
                        StructArray::from(resp_data),
                    )?;
                } else {
                    tracing::warn!("service response contained no data");
                }
            }
            Event::Stop(_) => break,
            _ => {}
        }
    }

    Ok(())
}

fn run_service_server(
    server: ros2_client::Server<BridgeServiceType>,
    request_type_info: TypeInfo<'static>,
    response_type_info: TypeInfo<'static>,
) -> eyre::Result<()> {
    let (mut node, adora_events) = AdoraNode::init_from_env()?;

    // receive_request_stream() borrows &server immutably;
    // async_send_response() also takes &self. Both are shared borrows, so
    // they can coexist in the same scope.
    let _deser_guard = TypeInfoGuard::deserialize(request_type_info);
    let request_stream = server.receive_request_stream().filter_map(|result| async {
        match result {
            Ok((req_id, msg)) => msg.0.map(|data| (req_id, data)),
            Err(e) => {
                tracing::warn!("ROS2 service request error: {e:?}");
                None
            }
        }
    });
    let merged = adora_events.merge_external(Box::pin(request_stream));

    // Map from request_id -> (ROS2 RmwRequestId, insertion time).
    // Handler responses carry the request_id in metadata, allowing out-of-order replies.
    // Stale entries are evicted after SERVICE_RESPONSE_TIMEOUT.
    let mut pending_requests: HashMap<
        String,
        (ros2_client::service::RmwRequestId, std::time::Instant),
    > = HashMap::new();
    let mut oldest_insert: Option<std::time::Instant> = None;

    for event in futures::executor::block_on_stream(merged) {
        match event {
            MergedEvent::Adora(Event::Input {
                id: _,
                metadata,
                data,
            }) => {
                let rid = get_string_param(&metadata.parameters, REQUEST_ID);
                if let Some(rid) = rid {
                    if let Some((rmw_id, _)) = pending_requests.remove(rid) {
                        let array_data = data.to_data();
                        let _ser_guard = TypeInfoGuard::serialize(response_type_info.clone());
                        futures::executor::block_on(
                            server.async_send_response(rmw_id, BridgeMessage(Some(array_data))),
                        )
                        .map_err(|e| eyre!("failed to send service response: {e:?}"))?;
                    } else {
                        tracing::warn!(
                            "response has request_id={rid} but no matching pending request"
                        );
                    }
                } else {
                    tracing::warn!("response input missing request_id metadata parameter");
                }
            }
            MergedEvent::Adora(Event::Stop(_)) => break,
            MergedEvent::Adora(_) => {}
            MergedEvent::External((rmw_id, data)) => {
                // Evict stale entries that never received a response.
                // Skip the O(n) scan when no entry could have expired yet.
                let now = std::time::Instant::now();
                if oldest_insert.is_some_and(|t| now.duration_since(t) >= SERVICE_RESPONSE_TIMEOUT)
                {
                    pending_requests.retain(|id, (_, t)| {
                        let keep = now.duration_since(*t) < SERVICE_RESPONSE_TIMEOUT;
                        if !keep {
                            tracing::warn!("evicting timed-out pending request {id}");
                        }
                        keep
                    });
                    oldest_insert = pending_requests.values().map(|(_, t)| *t).min();
                }

                if pending_requests.len() >= MAX_PENDING_REQUESTS {
                    tracing::warn!(
                        "pending requests queue full ({MAX_PENDING_REQUESTS}), dropping request"
                    );
                    continue;
                }
                let request_id = AdoraNode::new_request_id();
                if oldest_insert.is_none() {
                    oldest_insert = Some(now);
                }
                pending_requests.insert(request_id.clone(), (rmw_id, now));
                let mut params = adora_message::metadata::MetadataParameters::default();
                params.insert(REQUEST_ID.to_string(), Parameter::String(request_id));
                node.send_output("request".into(), params, StructArray::from(data))?;
            }
        }
    }

    Ok(())
}

// ---------------------------------------------------------------------------
// Action mode
// ---------------------------------------------------------------------------

fn run_action_mode(
    config: Ros2BridgeConfig,
    messages: Arc<HashMap<String, HashMap<String, Message>>>,
) -> eyre::Result<()> {
    let action_name = config.action.as_ref().unwrap();
    let action_type = config
        .action_type
        .as_ref()
        .context("action_type required")?;
    let role = config.role.as_ref().context("role required for action")?;
    let (package, type_name) = parse_type_str(action_type)?;

    let make_type_info = |suffix: &str| TypeInfo {
        package_name: Cow::Owned(package.clone()),
        message_name: Cow::Owned(format!("{type_name}_{suffix}")),
        messages: messages.clone(),
    };
    let goal_type_info = make_type_info("Goal");
    let result_type_info = make_type_info("Result");
    let feedback_type_info = make_type_info("Feedback");

    let (mut ros_node, _pool) = create_ros_node(&config)?;
    let qos = config.qos.to_rustdds_qos();
    let action_ros2_name = ros2_client::Name::new("/", action_name.trim_start_matches('/'))
        .map_err(|e| eyre!("failed to create action name: {e}"))?;
    let action_type_name = ros2_client::ActionTypeName::new(&package, &type_name);

    match role {
        Ros2Role::Client => {
            let action_qos = ros2_client::action::ActionClientQosPolicies {
                goal_service: qos.clone(),
                result_service: qos.clone(),
                cancel_service: qos.clone(),
                feedback_subscription: qos.clone(),
                status_subscription: qos,
            };

            let client = ros_node
                .create_action_client::<BridgeActionType>(
                    ros2_client::ServiceMapping::Enhanced,
                    &action_ros2_name,
                    &action_type_name,
                    action_qos,
                )
                .map_err(|e| eyre!("failed to create action client: {e:?}"))?;

            // NOTE: ros2_client does not provide a wait_for_action_server API.
            // The first goal send will time out (ACTION_GOAL_TIMEOUT) if the
            // action server is not yet available. Start the action server
            // before this dataflow for reliable operation.
            tracing::info!("action client created, waiting for goals from Adora inputs");

            run_action_client(client, goal_type_info, result_type_info, feedback_type_info)
        }
        Ros2Role::Server => {
            let action_qos = ros2_client::action::ActionServerQosPolicies {
                goal_service: qos.clone(),
                result_service: qos.clone(),
                cancel_service: qos.clone(),
                feedback_publisher: qos.clone(),
                status_publisher: qos,
            };

            let server = ros_node
                .create_action_server::<BridgeActionType>(
                    ros2_client::ServiceMapping::Enhanced,
                    &action_ros2_name,
                    &action_type_name,
                    action_qos,
                )
                .map_err(|e| eyre!("failed to create action server: {e:?}"))?;

            let async_server = ros2_client::action::AsyncActionServer::new(server);
            tracing::info!("action server created, waiting for goals from ROS2 clients");

            run_action_server(
                async_server,
                goal_type_info,
                result_type_info,
                feedback_type_info,
            )
        }
    }
}

fn run_action_client(
    client: ros2_client::action::ActionClient<BridgeActionType>,
    goal_type_info: TypeInfo<'static>,
    result_type_info: TypeInfo<'static>,
    feedback_type_info: TypeInfo<'static>,
) -> eyre::Result<()> {
    let (mut node, adora_events) = AdoraNode::init_from_env()?;
    let client = Arc::new(client);
    let in_flight = Arc::new(AtomicUsize::new(0));

    // Channel for feedback and result from background threads.
    // Bounded at 16 per concurrent goal to provide backpressure — if the main
    // loop can't keep up, feedback threads will block rather than consume
    // unbounded memory. `tx` is cloned per-goal; when the loop breaks on Stop,
    // background threads detect the closed channel and exit.
    let (tx, rx) = flume::bounded::<ActionEvent>(MAX_CONCURRENT_GOALS * 2);

    let rx_stream = rx.into_stream();
    let merged = adora_events.merge_external(Box::pin(rx_stream));

    for event in futures::executor::block_on_stream(merged) {
        match event {
            MergedEvent::Adora(Event::Input {
                id: _,
                metadata: _,
                data,
            }) => {
                // Cap concurrent in-flight goals
                if in_flight.load(Ordering::Relaxed) >= MAX_CONCURRENT_GOALS {
                    tracing::warn!(
                        "max concurrent goals ({MAX_CONCURRENT_GOALS}) reached, dropping goal"
                    );
                    continue;
                }

                let array_data = data.to_data();

                // Send goal with timeout (guard clears thread-local on drop)
                let _guard = TypeInfoGuard::serialize(goal_type_info.clone());
                let (goal_id, send_goal_response) = futures::executor::block_on(async {
                    let send = client.async_send_goal(BridgeMessage(Some(array_data)));
                    futures::pin_mut!(send);
                    let timeout = futures_timer::Delay::new(ACTION_GOAL_TIMEOUT);
                    match futures::future::select(send, timeout).await {
                        futures::future::Either::Left((result, _)) => {
                            result.map_err(|e| eyre!("failed to send action goal: {e:?}"))
                        }
                        futures::future::Either::Right(_) => {
                            eyre::bail!("action goal send timed out")
                        }
                    }
                })?;

                if !send_goal_response.accepted {
                    tracing::warn!("action goal was rejected by server");
                    continue;
                }

                in_flight.fetch_add(1, Ordering::Relaxed);

                // Spawn feedback reader thread
                let feedback_tx = tx.clone();
                let fb_type_info = feedback_type_info.clone();
                let client_ref = client.clone();
                std::thread::spawn(move || {
                    let _guard = TypeInfoGuard::deserialize(fb_type_info);
                    let stream = client_ref.feedback_stream(goal_id);
                    futures::pin_mut!(stream);
                    futures::executor::block_on(async {
                        while let Some(item) = stream.next().await {
                            match item {
                                Ok(msg) => {
                                    if let Some(data) = msg.0 {
                                        if feedback_tx.send(ActionEvent::Feedback(data)).is_err() {
                                            break;
                                        }
                                    } else {
                                        tracing::warn!("action feedback contained no data");
                                    }
                                }
                                Err(e) => {
                                    tracing::warn!("action feedback error: {e:?}");
                                }
                            }
                        }
                    });
                });

                // Spawn result reader thread
                let result_tx = tx.clone();
                let res_type_info = result_type_info.clone();
                let client_ref = client.clone();
                let in_flight_ref = in_flight.clone();
                std::thread::spawn(move || {
                    let _guard = TypeInfoGuard::deserialize(res_type_info);
                    let result = futures::executor::block_on(async {
                        let recv = client_ref.async_request_result(goal_id);
                        futures::pin_mut!(recv);
                        let timeout = futures_timer::Delay::new(ACTION_RESULT_TIMEOUT);
                        match futures::future::select(recv, timeout).await {
                            futures::future::Either::Left((result, _)) => Some(result),
                            futures::future::Either::Right(_) => {
                                tracing::warn!("action result timed out");
                                None
                            }
                        }
                    });
                    if let Some(Ok((_status, msg))) = result {
                        if let Some(data) = msg.0 {
                            let _ = result_tx.send(ActionEvent::Result(data));
                        } else {
                            tracing::warn!("action result contained no data");
                        }
                    } else if let Some(Err(e)) = result {
                        tracing::warn!("action result error: {e:?}");
                    }
                    in_flight_ref.fetch_sub(1, Ordering::Relaxed);
                });
            }
            MergedEvent::Adora(Event::Stop(_)) => break,
            MergedEvent::Adora(_) => {}
            MergedEvent::External(action_event) => match action_event {
                ActionEvent::Feedback(data) => {
                    node.send_output(
                        "feedback".into(),
                        Default::default(),
                        StructArray::from(data),
                    )?;
                }
                ActionEvent::Result(data) => {
                    node.send_output("result".into(), Default::default(), StructArray::from(data))?;
                }
            },
        }
    }

    Ok(())
}

enum ActionEvent {
    Feedback(ArrayData),
    Result(ArrayData),
}

enum ServerEvent {
    NewGoal {
        goal_id: String,
        handle: ros2_client::action::ExecutingGoalHandle<BridgeMessage>,
        data: ArrayData,
    },
    /// Goal accepted but contained no data; main loop should send Aborted.
    AbortGoal {
        goal_id: String,
        handle: ros2_client::action::ExecutingGoalHandle<BridgeMessage>,
    },
}

fn run_action_server(
    server: ros2_client::action::AsyncActionServer<BridgeActionType>,
    goal_type_info: TypeInfo<'static>,
    result_type_info: TypeInfo<'static>,
    feedback_type_info: TypeInfo<'static>,
) -> eyre::Result<()> {
    let (mut node, adora_events) = AdoraNode::init_from_env()?;
    let server = Arc::new(server);

    // Map goal_id -> ExecutingGoalHandle for dispatching feedback/result
    let mut executing_goals: HashMap<
        String,
        ros2_client::action::ExecutingGoalHandle<BridgeMessage>,
    > = HashMap::new();

    // Channel for new goals from the background receive thread.
    // Bounded to provide backpressure: if the main loop can't keep up,
    // the receive thread will block.
    let (tx, rx) = flume::bounded::<ServerEvent>(MAX_CONCURRENT_GOALS * 2);

    // Background thread: receive goals from ROS2 clients, auto-accept and
    // start executing, then forward to the main loop.
    let server_ref = server.clone();
    std::thread::spawn(move || {
        let _deser_guard = TypeInfoGuard::deserialize(goal_type_info);
        futures::executor::block_on(async {
            loop {
                let new_goal_handle = match server_ref.receive_new_goal().await {
                    Ok(h) => h,
                    Err(e) => {
                        tracing::warn!("failed to receive new goal: {e:?}");
                        continue;
                    }
                };

                let goal_data = server_ref.get_new_goal(new_goal_handle.clone());

                let accepted = match server_ref.accept_goal(new_goal_handle).await {
                    Ok(h) => h,
                    Err(e) => {
                        tracing::warn!("failed to accept goal: {e:?}");
                        continue;
                    }
                };

                let executing = match server_ref.start_executing_goal(accepted).await {
                    Ok(h) => h,
                    Err(e) => {
                        tracing::warn!("failed to start executing goal: {e:?}");
                        continue;
                    }
                };

                let goal_id = executing.goal_id().uuid.to_string();

                let data = match goal_data {
                    Some(BridgeMessage(Some(d))) => d,
                    _ => {
                        tracing::warn!("goal {goal_id} contained no data, aborting");
                        let _ = tx.send(ServerEvent::AbortGoal {
                            goal_id,
                            handle: executing,
                        });
                        continue;
                    }
                };

                if tx
                    .send(ServerEvent::NewGoal {
                        goal_id,
                        handle: executing,
                        data,
                    })
                    .is_err()
                {
                    break;
                }
            }
        });
    });

    let rx_stream = rx.into_stream();
    let merged = adora_events.merge_external(Box::pin(rx_stream));

    for event in futures::executor::block_on_stream(merged) {
        match event {
            MergedEvent::Adora(Event::Input { id, metadata, data }) => {
                let goal_id = match get_string_param(&metadata.parameters, GOAL_ID) {
                    Some(s) => s.to_owned(),
                    None => {
                        tracing::warn!("action server input `{id}` missing goal_id in metadata");
                        continue;
                    }
                };

                match id.as_str() {
                    "feedback" => {
                        if let Some(handle) = executing_goals.get(&goal_id) {
                            let handle = handle.clone();
                            let array_data = data.to_data();
                            let _guard = TypeInfoGuard::serialize(feedback_type_info.clone());
                            if let Err(e) = futures::executor::block_on(
                                server.publish_feedback(handle, BridgeMessage(Some(array_data))),
                            ) {
                                tracing::warn!(
                                    "failed to publish feedback for goal {goal_id}: {e:?}"
                                );
                            }
                        } else {
                            tracing::warn!("feedback for unknown goal_id: {goal_id}");
                        }
                    }
                    "result" => {
                        if let Some(handle) = executing_goals.remove(&goal_id) {
                            let array_data = data.to_data();
                            let status = match get_string_param(&metadata.parameters, GOAL_STATUS) {
                                Some(s) => match s {
                                    GOAL_STATUS_SUCCEEDED => {
                                        ros2_client::action::GoalEndStatus::Succeeded
                                    }
                                    GOAL_STATUS_ABORTED => {
                                        ros2_client::action::GoalEndStatus::Aborted
                                    }
                                    GOAL_STATUS_CANCELED => {
                                        ros2_client::action::GoalEndStatus::Canceled
                                    }
                                    other => {
                                        tracing::warn!(
                                            "unknown goal_status `{other}` for goal {goal_id}, \
                                             defaulting to Aborted"
                                        );
                                        ros2_client::action::GoalEndStatus::Aborted
                                    }
                                },
                                None => ros2_client::action::GoalEndStatus::Succeeded,
                            };
                            let _guard = TypeInfoGuard::serialize(result_type_info.clone());
                            let send_result = server.send_result_response(
                                handle,
                                status,
                                BridgeMessage(Some(array_data)),
                            );
                            let result = futures::executor::block_on(async {
                                futures::pin_mut!(send_result);
                                let timeout = futures_timer::Delay::new(ACTION_RESULT_TIMEOUT);
                                match futures::future::select(send_result, timeout).await {
                                    futures::future::Either::Left((r, _)) => Some(r),
                                    futures::future::Either::Right(_) => None,
                                }
                            });
                            match result {
                                Some(Err(e)) => {
                                    tracing::warn!(
                                        "failed to send result for goal {goal_id}: {e:?}"
                                    );
                                }
                                None => {
                                    tracing::warn!("result response timed out for goal {goal_id}");
                                }
                                _ => {}
                            }
                        } else {
                            tracing::warn!("result for unknown goal_id: {goal_id}");
                        }
                    }
                    other => {
                        tracing::warn!("unexpected input `{other}` on action server");
                    }
                }
            }
            MergedEvent::Adora(Event::Stop(_)) => break,
            MergedEvent::Adora(_) => {}
            MergedEvent::External(server_event) => match server_event {
                ServerEvent::AbortGoal { goal_id, handle } => {
                    abort_executing_goal(&server, &result_type_info, handle, &goal_id);
                }
                ServerEvent::NewGoal {
                    goal_id,
                    handle,
                    data,
                } => {
                    if executing_goals.len() >= MAX_CONCURRENT_GOALS {
                        tracing::warn!(
                            "max concurrent goals ({MAX_CONCURRENT_GOALS}) reached, \
                             aborting goal {goal_id}"
                        );
                        abort_executing_goal(&server, &result_type_info, handle, &goal_id);
                        continue;
                    }

                    let mut parameters = BTreeMap::new();
                    parameters.insert(GOAL_ID.to_string(), Parameter::String(goal_id.clone()));
                    if let Err(e) =
                        node.send_output("goal".into(), parameters, StructArray::from(data))
                    {
                        tracing::warn!("failed to send goal {goal_id} to handler: {e}");
                        abort_executing_goal(&server, &result_type_info, handle, &goal_id);
                        continue;
                    }
                    executing_goals.insert(goal_id, handle);
                }
            },
        }
    }

    Ok(())
}

/// Send an Aborted result response for an executing goal so the ROS2 client
/// doesn't hang indefinitely.
fn abort_executing_goal(
    server: &ros2_client::action::AsyncActionServer<BridgeActionType>,
    result_type_info: &TypeInfo<'static>,
    handle: ros2_client::action::ExecutingGoalHandle<BridgeMessage>,
    goal_id: &str,
) {
    let _guard = TypeInfoGuard::serialize(result_type_info.clone());
    let send = server.send_result_response(
        handle,
        ros2_client::action::GoalEndStatus::Aborted,
        BridgeMessage(None),
    );
    if let Err(e) = futures::executor::block_on(send) {
        tracing::warn!("failed to send abort for goal {goal_id}: {e:?}");
    }
}

// ---------------------------------------------------------------------------
// Topic event loops (existing)
// ---------------------------------------------------------------------------

fn run_publish_only(
    _node: AdoraNode,
    adora_events: adora_node_api::EventStream,
    publishers: Vec<(
        String,
        TypeInfo<'static>,
        ros2_client::Publisher<TypedValue<'static>>,
    )>,
    messages: &Arc<HashMap<String, HashMap<String, Message>>>,
) -> eyre::Result<()> {
    for event in futures::executor::block_on_stream(adora_events) {
        match event {
            Event::Input {
                id,
                metadata: _,
                data,
            } => {
                handle_publish_input(&id, &data, &publishers, messages)?;
            }
            Event::Stop(_) => break,
            _ => {}
        }
    }
    Ok(())
}

fn run_single_subscriber(
    mut node: AdoraNode,
    adora_events: adora_node_api::EventStream,
    output_id: String,
    sub: SubscriptionStream,
    publishers: Vec<(
        String,
        TypeInfo<'static>,
        ros2_client::Publisher<TypedValue<'static>>,
    )>,
    messages: &Arc<HashMap<String, HashMap<String, Message>>>,
) -> eyre::Result<()> {
    let ros_stream = sub
        .subscription
        .async_stream_seed(sub.deserializer)
        .filter_map(|result| async {
            match result {
                Ok((data, _info)) => Some(data),
                Err(e) => {
                    tracing::warn!("ROS2 subscription error: {e:?}");
                    None
                }
            }
        });
    let merged = adora_events.merge_external(Box::pin(ros_stream));

    for event in futures::executor::block_on_stream(merged) {
        match event {
            MergedEvent::Adora(Event::Input {
                id,
                metadata: _,
                data,
            }) => {
                handle_publish_input(&id, &data, &publishers, messages)?;
            }
            MergedEvent::Adora(Event::Stop(_)) => break,
            MergedEvent::Adora(_) => {}
            MergedEvent::External(data) => {
                node.send_output(
                    output_id.clone().into(),
                    Default::default(),
                    StructArray::from(data),
                )?;
            }
        }
    }
    Ok(())
}

fn run_multi_subscriber(
    mut node: AdoraNode,
    adora_events: adora_node_api::EventStream,
    subscribers: Vec<(String, SubscriptionStream)>,
    publishers: Vec<(
        String,
        TypeInfo<'static>,
        ros2_client::Publisher<TypedValue<'static>>,
    )>,
    messages: &Arc<HashMap<String, HashMap<String, Message>>>,
) -> eyre::Result<()> {
    let (tx, rx) = flume::bounded::<(String, ArrayData)>(10);

    let mut handles = Vec::new();
    for (output_id, sub) in subscribers {
        let tx = tx.clone();
        handles.push(std::thread::spawn(move || {
            let stream = Box::pin(
                sub.subscription
                    .async_stream_seed(sub.deserializer)
                    .filter_map(|result| async {
                        match result {
                            Ok((data, _info)) => Some(data),
                            Err(e) => {
                                tracing::warn!("ROS2 subscription error: {e:?}");
                                None
                            }
                        }
                    }),
            );
            for data in futures::executor::block_on_stream(stream) {
                if tx.send((output_id.clone(), data)).is_err() {
                    break;
                }
            }
        }));
    }
    drop(tx);

    let rx_stream = rx.into_stream();
    let merged = adora_events.merge_external(Box::pin(rx_stream));

    for event in futures::executor::block_on_stream(merged) {
        match event {
            MergedEvent::Adora(Event::Input {
                id,
                metadata: _,
                data,
            }) => {
                handle_publish_input(&id, &data, &publishers, messages)?;
            }
            MergedEvent::Adora(Event::Stop(_)) => break,
            MergedEvent::Adora(_) => {}
            MergedEvent::External((output_id, data)) => {
                node.send_output(
                    output_id.into(),
                    Default::default(),
                    StructArray::from(data),
                )?;
            }
        }
    }

    for handle in handles {
        if let Err(e) = handle.join() {
            tracing::warn!("subscriber thread panicked: {e:?}");
        }
    }
    Ok(())
}

fn handle_publish_input(
    input_id: &str,
    data: &arrow::array::ArrayRef,
    publishers: &[(
        String,
        TypeInfo<'static>,
        ros2_client::Publisher<TypedValue<'static>>,
    )],
    _messages: &Arc<HashMap<String, HashMap<String, Message>>>,
) -> eyre::Result<()> {
    for (pub_input_id, type_info, publisher) in publishers {
        if pub_input_id == input_id {
            let typed_value = TypedValue {
                value: data,
                type_info,
            };
            publisher
                .publish(typed_value)
                .map_err(|e| eyre!("failed to publish to ROS2: {e:?}"))?;
        }
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

struct SubscriptionStream {
    deserializer: StructDeserializer<'static>,
    subscription: ros2_client::Subscription<ArrayData>,
}

fn create_ros_node(
    config: &Ros2BridgeConfig,
) -> eyre::Result<(ros2_client::Node, futures::executor::ThreadPool)> {
    let ros_context =
        ros2_client::Context::new().map_err(|e| eyre!("failed to create ROS2 context: {e:?}"))?;
    let node_name = config
        .node_name
        .clone()
        .unwrap_or_else(|| "adora_ros2_bridge".to_string());
    let namespace = &config.namespace;
    let mut ros_node = ros_context
        .new_node(
            ros2_client::NodeName::new(namespace, &node_name)
                .map_err(|e| eyre!("failed to create ROS2 node name: {e}"))?,
            ros2_client::NodeOptions::new().enable_rosout(true),
        )
        .map_err(|e| eyre!("failed to create ROS2 node: {e:?}"))?;

    let pool = futures::executor::ThreadPool::new()?;
    let spinner = ros_node
        .spinner()
        .map_err(|e| eyre!("failed to create spinner: {e:?}"))?;
    pool.spawn(async {
        if let Err(err) = spinner.spin().await {
            tracing::error!("ros2 spinner failed: {err:?}");
        }
    })
    .context("failed to spawn ros2 spinner")?;

    Ok((ros_node, pool))
}

/// Wait for a ROS2 service to become available.
///
/// NOTE: This blocks the main thread and does not respond to Adora Stop events.
/// If the service never appears, the loop will exhaust its 10 retries and bail.
fn wait_for_service(
    client: &ros2_client::Client<BridgeServiceType>,
    ros_node: &ros2_client::Node,
) -> eyre::Result<()> {
    let service_ready = async {
        for _ in 0..10 {
            let ready = client.wait_for_service(ros_node);
            futures::pin_mut!(ready);
            let timeout = futures_timer::Delay::new(Duration::from_secs(2));
            match futures::future::select(ready, timeout).await {
                futures::future::Either::Left(((), _)) => {
                    tracing::info!("service is ready");
                    return Ok(());
                }
                futures::future::Either::Right(_) => {
                    tracing::info!("timeout waiting for service, retrying");
                }
            }
        }
        eyre::bail!("service not available after 10 retries");
    };
    futures::executor::block_on(service_ready)
}

fn load_messages() -> eyre::Result<Arc<HashMap<String, HashMap<String, Message>>>> {
    let ament_prefix_path = std::env::var("AMENT_PREFIX_PATH").unwrap_or_default();
    let paths: Vec<std::path::PathBuf> = if ament_prefix_path.is_empty() {
        vec![]
    } else {
        std::env::split_paths(&ament_prefix_path).collect()
    };
    let path_refs: Vec<&Path> = paths.iter().map(|p| p.as_path()).collect();

    let packages = adora_ros2_bridge_msg_gen::get_packages(&path_refs)
        .map_err(|e| eyre!("failed to parse ROS2 message definitions: {e:?}"))?;

    let mut messages: HashMap<String, HashMap<String, Message>> = HashMap::new();
    for package in packages {
        let mut package_messages = HashMap::new();
        for message in &package.messages {
            package_messages.insert(message.name.clone(), message.clone());
        }
        for service in &package.services {
            package_messages.insert(service.request.name.clone(), service.request.clone());
            package_messages.insert(service.response.name.clone(), service.response.clone());
        }
        for action in &package.actions {
            package_messages.insert(action.goal.name.clone(), action.goal.clone());
            package_messages.insert(action.result.name.clone(), action.result.clone());
            package_messages.insert(action.feedback.name.clone(), action.feedback.clone());
        }
        messages.insert(package.name.clone(), package_messages);
    }

    if messages.is_empty() {
        tracing::error!(
            "no ROS2 message definitions found - bridge will fail to serialize/deserialize. \
             Ensure AMENT_PREFIX_PATH is set and points to a sourced ROS2 workspace."
        );
    }

    Ok(Arc::new(messages))
}

fn parse_type_str(type_str: &str) -> eyre::Result<(String, String)> {
    let (pkg, type_name) = type_str
        .split_once('/')
        .filter(|(p, t)| !p.is_empty() && !t.is_empty() && !t.contains('/'))
        .ok_or_else(|| eyre!("invalid type format: {type_str}, expected package/TypeName"))?;
    for (label, part) in [("package", pkg), ("type name", type_name)] {
        if !part.chars().all(|c| c.is_ascii_alphanumeric() || c == '_') {
            eyre::bail!(
                "invalid {label} `{part}` in type `{type_str}`: \
                 only ASCII alphanumeric and underscore allowed"
            );
        }
    }
    Ok((pkg.to_string(), type_name.to_string()))
}

fn resolve_topics(config: &Ros2BridgeConfig) -> eyre::Result<Vec<Ros2TopicConfig>> {
    match (&config.topic, &config.topics) {
        (Some(topic), None) => {
            let message_type = config
                .message_type
                .clone()
                .context("message_type required with single topic")?;
            Ok(vec![Ros2TopicConfig {
                topic: topic.clone(),
                message_type,
                direction: config.direction.clone(),
                output: None,
                input: None,
                qos: None,
            }])
        }
        (None, Some(topics)) => Ok(topics.clone()),
        _ => eyre::bail!("topic mode requires exactly one of `topic` or `topics` to be set"),
    }
}

trait ToRustddsQos {
    fn to_rustdds_qos(&self) -> rustdds::QosPolicies;
}

impl ToRustddsQos for Ros2QosConfig {
    fn to_rustdds_qos(&self) -> rustdds::QosPolicies {
        let mut builder = rustdds::QosPolicyBuilder::new();

        let durability = match self.durability.as_deref() {
            Some("transient_local") => rustdds::policy::Durability::TransientLocal,
            _ => rustdds::policy::Durability::Volatile,
        };
        builder = builder.durability(durability);

        if self.reliable {
            let max_blocking = self
                .max_blocking_time
                .map(ros2_client::ros2::Duration::from_frac_seconds)
                .unwrap_or(ros2_client::ros2::Duration::from_millis(100));
            builder = builder.reliability(rustdds::policy::Reliability::Reliable {
                max_blocking_time: max_blocking,
            });
        } else {
            builder = builder.reliability(rustdds::policy::Reliability::BestEffort);
        }

        if self.keep_all {
            builder = builder.history(rustdds::policy::History::KeepAll);
        } else {
            let depth = self.keep_last.unwrap_or(1);
            builder = builder.history(rustdds::policy::History::KeepLast {
                depth: depth.max(1),
            });
        }

        let liveliness = match self.liveliness.as_deref() {
            Some("manual_by_participant") => rustdds::policy::Liveliness::ManualByParticipant {
                lease_duration: lease_duration_from_secs(self.lease_duration),
            },
            Some("manual_by_topic") => rustdds::policy::Liveliness::ManualByTopic {
                lease_duration: lease_duration_from_secs(self.lease_duration),
            },
            _ => rustdds::policy::Liveliness::Automatic {
                lease_duration: lease_duration_from_secs(self.lease_duration),
            },
        };
        builder = builder.liveliness(liveliness);

        builder.build()
    }
}

fn lease_duration_from_secs(secs: Option<f64>) -> ros2_client::ros2::Duration {
    match secs {
        Some(s) => ros2_client::ros2::Duration::from_frac_seconds(s),
        None => ros2_client::ros2::Duration::INFINITE,
    }
}
