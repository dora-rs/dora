use std::{
    borrow::Cow,
    collections::{BTreeMap, HashMap},
    path::Path,
    sync::Arc,
    time::Duration,
};

use arrow::array::{ArrayData, StructArray};
use dora_message::{
    descriptor::{
        Ros2BridgeConfig, Ros2Direction, Ros2QosConfig, Ros2Role, Ros2TopicConfig,
        Ros2TransportConfig,
    },
    metadata::{Parameter, get_string_param},
};
use dora_node_api::{
    DoraNode, Event,
    merged::{MergeExternal, MergedEvent},
};
use dora_ros2_bridge::transport::action::zenoh::{
    ActionClient as ZenohActionClient, ActionKeys, ActionServer as ZenohActionServer, ActionTokens,
};
use dora_ros2_bridge::transport::action::{ConcurrentGoalLimit, GoalSlots, MAX_CONCURRENT_GOALS};
use dora_ros2_bridge::transport::zenoh::{
    Context as ZenohContext, ContextOptions as ZenohContextOptions,
    compatibility::{TypeDescriptionResolver, resolve_action, resolve_message, resolve_service},
    keyexpr::{DataKey, TopicToken},
    pubsub::{NodePublisher, NodeSubscription, PubSubError},
    qos::ZenohQosMapping,
    service::{NodeServiceClient, NodeServiceServer},
};
use dora_ros2_bridge::transport::{Durability, History, Liveliness, Reliability, Ros2Qos};
use dora_ros2_bridge::{ros2_client, rustdds};
use dora_ros2_bridge_arrow::{
    BridgeActionType, BridgeMessage, BridgeServiceType, TypeInfo, TypeInfoGuard, TypedValue,
    deserialize::StructDeserializer, deserialize_cdr, deserialize_raw_cdr, serialize_cdr,
    serialize_raw_cdr,
};
use dora_ros2_bridge_msg_gen::types::Message;
use eyre::{Context, ContextCompat, eyre};
use futures::{StreamExt, task::SpawnExt};

/// Maximum pending service requests before dropping new ones.
const MAX_PENDING_REQUESTS: usize = 64;

fn peer_value_or_warn<T, E: std::fmt::Display>(result: Result<T, E>, operation: &str) -> Option<T> {
    match result {
        Ok(value) => Some(value),
        Err(error) => {
            tracing::warn!("{operation}: {error}");
            None
        }
    }
}

use dora_message::metadata::{
    GOAL_ID, GOAL_STATUS, GOAL_STATUS_ABORTED, GOAL_STATUS_CANCELED, GOAL_STATUS_SUCCEEDED,
    REQUEST_ID,
};

fn main() -> eyre::Result<()> {
    tracing_subscriber::fmt::init();

    let config_json = std::env::var("DORA_ROS2_BRIDGE_CONFIG")
        .context("missing DORA_ROS2_BRIDGE_CONFIG env var")?;
    let config: Ros2BridgeConfig =
        serde_json::from_str(&config_json).context("failed to parse ROS2 bridge config")?;

    let messages = load_messages()?;

    if matches!(config.transport, Ros2TransportConfig::Zenoh { .. }) {
        return run_zenoh_mode(config, messages);
    }

    if std::env::var("RMW_IMPLEMENTATION").ok().as_deref() == Some("rmw_zenoh_cpp") {
        tracing::warn!(
            "DDS transport selected while RMW_IMPLEMENTATION=rmw_zenoh_cpp; set transport.kind: zenoh explicitly"
        );
    }

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

fn run_zenoh_mode(
    config: Ros2BridgeConfig,
    messages: Arc<HashMap<String, HashMap<String, Message>>>,
) -> eyre::Result<()> {
    let Ros2TransportConfig::Zenoh {
        compatibility,
        config_uri,
    } = &config.transport
    else {
        unreachable!()
    };
    tracing::info!(profile = ?compatibility, config_source = if config_uri.is_some() { "explicit" } else { "environment-or-default" }, "starting native ROS2 Zenoh transport");
    let domain_id = std::env::var("ROS_DOMAIN_ID")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(0);
    let context = futures::executor::block_on(ZenohContext::open(ZenohContextOptions {
        domain_id,
        config_uri: config_uri
            .as_ref()
            .map(|path| path.to_string_lossy().into_owned()),
    }))
    .map_err(|error| eyre!(error))?;
    let node_name = config.node_name.as_deref().unwrap_or("dora_ros2_bridge");
    let node = futures::executor::block_on(context.create_node(
        &context.zid(),
        node_name,
        "/",
        &config.namespace,
        node_name,
    ))
    .map_err(|error| eyre!(error))?;
    if config.service.is_some() {
        return run_zenoh_service_mode(&config, &node, messages);
    }
    if config.action.is_some() {
        return run_zenoh_action_mode(&config, &node, messages);
    }
    run_zenoh_topic_mode(&config, &node, messages)
}

struct ZenohTopicPublisher {
    input: String,
    type_info: TypeInfo<'static>,
    publisher: NodePublisher,
}

fn run_zenoh_topic_mode(
    config: &Ros2BridgeConfig,
    zenoh_node: &dora_ros2_bridge::transport::zenoh::Node,
    messages: Arc<HashMap<String, HashMap<String, Message>>>,
) -> eyre::Result<()> {
    let Ros2TransportConfig::Zenoh { compatibility, .. } = config.transport else {
        unreachable!()
    };
    let resolver = TypeDescriptionResolver::from_ament_prefix_path();
    let domain_id = std::env::var("ROS_DOMAIN_ID")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(0);
    let mut publishers = Vec::new();
    let mut subscribers = Vec::new();
    for topic in resolve_topics(config)? {
        let (package, message_name) = parse_type_str(&topic.message_type)?;
        let identity = resolve_message(compatibility, &package, &message_name, &resolver)?;
        let key = DataKey::new(domain_id, &topic.topic, &identity)?;
        let qos = topic.qos.as_ref().unwrap_or(&config.qos).to_neutral_qos();
        let token = TopicToken {
            name: topic.topic.clone(),
            type_name: identity.dds_name.clone(),
            type_hash: identity.key_hash_component(),
            qos: ZenohQosMapping::from_ros_qos(&qos).to_string(),
        };
        let type_info = TypeInfo {
            package_name: Cow::Owned(package),
            message_name: Cow::Owned(message_name),
            messages: messages.clone(),
        };
        match topic.direction {
            Ros2Direction::Publish => {
                let publisher = futures::executor::block_on(NodePublisher::declare(
                    zenoh_node,
                    key.as_str(),
                    token,
                    &qos,
                ))?;
                publishers.push(ZenohTopicPublisher {
                    input: topic
                        .input
                        .unwrap_or_else(|| topic.topic.trim_start_matches('/').replace('/', "_")),
                    type_info,
                    publisher,
                });
            }
            Ros2Direction::Subscribe => {
                let decode_info = type_info.clone();
                let decoder = Arc::new(move |bytes: &[u8]| {
                    deserialize_raw_cdr(bytes, decode_info.clone(), 64 * 1024 * 1024)
                        .map_err(|error| PubSubError::Decode(error.to_string()))
                });
                let subscription = futures::executor::block_on(NodeSubscription::declare(
                    zenoh_node,
                    key.as_str(),
                    token,
                    &qos,
                    64,
                    64 * 1024 * 1024,
                    decoder,
                ))?;
                subscribers.push((
                    topic
                        .output
                        .unwrap_or_else(|| topic.topic.trim_start_matches('/').replace('/', "_")),
                    subscription,
                ));
            }
        }
    }
    let (mut node, dora_events) = DoraNode::init_from_env()?;
    let (tx, rx) = flume::bounded(64);
    for (output, subscription) in subscribers {
        let tx = tx.clone();
        std::thread::spawn(move || {
            futures::executor::block_on(async move {
                while let Ok((data, _metadata)) = subscription.recv_async().await {
                    if tx.send((output.clone(), data)).is_err() {
                        break;
                    }
                }
            })
        });
    }
    drop(tx);
    let merged = dora_events.merge_external(Box::pin(rx.into_stream()));
    for event in futures::executor::block_on_stream(merged) {
        match event {
            MergedEvent::Dora(Event::Input { id, data, .. }) => {
                for publisher in &publishers {
                    if publisher.input == id.as_str() {
                        let value = TypedValue {
                            value: &data,
                            type_info: &publisher.type_info,
                        };
                        let cdr = serialize_raw_cdr(&value)?;
                        futures::executor::block_on(publisher.publisher.publish(&cdr))?;
                    }
                }
            }
            MergedEvent::Dora(Event::Stop(_)) => break,
            MergedEvent::Dora(_) => {}
            MergedEvent::External((output, data)) => {
                node.send_output(output.into(), Default::default(), StructArray::from(data))?
            }
        }
    }
    Ok(())
}

fn run_zenoh_service_mode(
    config: &Ros2BridgeConfig,
    zenoh_node: &dora_ros2_bridge::transport::zenoh::Node,
    messages: Arc<HashMap<String, HashMap<String, Message>>>,
) -> eyre::Result<()> {
    let Ros2TransportConfig::Zenoh { compatibility, .. } = config.transport else {
        unreachable!()
    };
    let name = config.service.as_deref().context("service name required")?;
    let (package, service_name) = parse_type_str(
        config
            .service_type
            .as_deref()
            .context("service_type required")?,
    )?;
    let identity = resolve_service(
        compatibility,
        &package,
        &service_name,
        &TypeDescriptionResolver::from_ament_prefix_path(),
    )?;
    let domain_id = std::env::var("ROS_DOMAIN_ID")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(0);
    let key = DataKey::new(domain_id, name, &identity)?;
    let qos = config.qos.to_neutral_qos();
    let token = TopicToken {
        name: name.into(),
        type_name: identity.dds_name.clone(),
        type_hash: identity.key_hash_component(),
        qos: ZenohQosMapping::from_ros_qos(&qos).to_string(),
    };
    let request_info = TypeInfo {
        package_name: Cow::Owned(package.clone()),
        message_name: Cow::Owned(format!("{service_name}_Request")),
        messages: messages.clone(),
    };
    let response_info = TypeInfo {
        package_name: Cow::Owned(package),
        message_name: Cow::Owned(format!("{service_name}_Response")),
        messages,
    };
    match config.role.as_ref().context("role required for service")? {
        Ros2Role::Client => {
            let client = futures::executor::block_on(NodeServiceClient::declare(
                zenoh_node,
                key.as_str(),
                token,
                MAX_PENDING_REQUESTS,
            ))?;
            let (mut node, events) = DoraNode::init_from_env()?;
            for event in futures::executor::block_on_stream(events) {
                match event {
                    Event::Input { data, .. } => {
                        let request = serialize_raw_cdr(&TypedValue {
                            value: &data,
                            type_info: &request_info,
                        })?;
                        let Some(response) = peer_value_or_warn(
                            futures::executor::block_on(
                                client.call(request, SERVICE_RESPONSE_TIMEOUT),
                            ),
                            "Zenoh service call failed",
                        ) else {
                            continue;
                        };
                        let Some(data) = peer_value_or_warn(
                            deserialize_raw_cdr(&response, response_info.clone(), 64 * 1024 * 1024),
                            "invalid Zenoh service response",
                        ) else {
                            continue;
                        };
                        node.send_output(
                            "response".into(),
                            Default::default(),
                            StructArray::from(data),
                        )?;
                    }
                    Event::Stop(_) => break,
                    _ => {}
                }
            }
        }
        Ros2Role::Server => {
            let server = Arc::new(futures::executor::block_on(NodeServiceServer::declare(
                zenoh_node,
                key.as_str(),
                token,
                MAX_PENDING_REQUESTS,
                SERVICE_RESPONSE_TIMEOUT,
            ))?);
            let (mut node, events) = DoraNode::init_from_env()?;
            let (tx, rx) = flume::bounded(MAX_PENDING_REQUESTS);
            let receiver = server.clone();
            std::thread::spawn(move || {
                futures::executor::block_on(async move {
                    while let Ok(request) = receiver.recv().await {
                        if tx.send(request).is_err() {
                            break;
                        }
                    }
                })
            });
            let mut pending = HashMap::new();
            let merged = events.merge_external(Box::pin(rx.into_stream()));
            for event in futures::executor::block_on_stream(merged) {
                match event {
                    MergedEvent::External(request) => {
                        let request_key = format!(
                            "{}:{}",
                            request.id.sequence_number,
                            hex_gid(&request.id.client_gid)
                        );
                        let Some(data) = peer_value_or_warn(
                            deserialize_raw_cdr(
                                &request.payload,
                                request_info.clone(),
                                64 * 1024 * 1024,
                            ),
                            "invalid Zenoh service request",
                        ) else {
                            futures::executor::block_on(
                                server.reject(request.id, "invalid request payload"),
                            )?;
                            continue;
                        };
                        pending.insert(request_key.clone(), request.id);
                        let mut metadata = dora_message::metadata::MetadataParameters::default();
                        metadata.insert(REQUEST_ID.into(), Parameter::String(request_key));
                        node.send_output("request".into(), metadata, StructArray::from(data))?;
                    }
                    MergedEvent::Dora(Event::Input { metadata, data, .. }) => {
                        if let Some(key) = get_string_param(&metadata.parameters, REQUEST_ID)
                            && let Some(id) = pending.remove(key)
                        {
                            let response = serialize_raw_cdr(&TypedValue {
                                value: &data,
                                type_info: &response_info,
                            })?;
                            // A slow local handler can let the request expire before we
                            // reply; a RequestExpired/RequestAbsent here must not tear
                            // down the whole service bridge.
                            if let Err(error) =
                                futures::executor::block_on(server.reply(id, &response))
                            {
                                tracing::warn!("failed to reply to Zenoh service request: {error}");
                            }
                        }
                    }
                    MergedEvent::Dora(Event::Stop(_)) => break,
                    MergedEvent::Dora(_) => {}
                }
            }
        }
    }
    Ok(())
}

fn hex_gid(gid: &[u8; 16]) -> String {
    gid.iter().map(|byte| format!("{byte:02x}")).collect()
}

fn run_zenoh_action_mode(
    config: &Ros2BridgeConfig,
    zenoh_node: &dora_ros2_bridge::transport::zenoh::Node,
    messages: Arc<HashMap<String, HashMap<String, Message>>>,
) -> eyre::Result<()> {
    let Ros2TransportConfig::Zenoh { compatibility, .. } = config.transport else {
        unreachable!()
    };
    let action_name = config.action.as_deref().context("action name required")?;
    let (package, action_type) = parse_type_str(
        config
            .action_type
            .as_deref()
            .context("action_type required")?,
    )?;
    let resolver = TypeDescriptionResolver::from_ament_prefix_path();
    let identities = resolve_action(compatibility, &package, &action_type, &resolver)?;
    let find = |suffix: &str| {
        identities
            .iter()
            .find(|identity| identity.ros_name.ends_with(suffix))
            .cloned()
            .with_context(|| format!("missing action identity {suffix}"))
    };
    let cancel = resolve_service(compatibility, "action_msgs", "CancelGoal", &resolver)?;
    let status = resolve_message(compatibility, "action_msgs", "GoalStatusArray", &resolver)?;
    let domain = std::env::var("ROS_DOMAIN_ID")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(0);
    let endpoints = dora_ros2_bridge::transport::action::ActionEndpoints::new(
        action_name,
        &package,
        &action_type,
    );
    let send_goal_identity = find("_SendGoal")?;
    let get_result_identity = find("_GetResult")?;
    let feedback_identity = find("_FeedbackMessage")?;
    let keys = ActionKeys {
        send_goal: DataKey::new(domain, &endpoints.send_goal.name, &send_goal_identity)?
            .as_str()
            .into(),
        get_result: DataKey::new(domain, &endpoints.get_result.name, &get_result_identity)?
            .as_str()
            .into(),
        cancel_goal: DataKey::new(domain, &endpoints.cancel_goal.name, &cancel)?
            .as_str()
            .into(),
        feedback: DataKey::new(domain, &endpoints.feedback.name, &feedback_identity)?
            .as_str()
            .into(),
        status: DataKey::new(domain, &endpoints.status.name, &status)?
            .as_str()
            .into(),
    };
    let goal_info = TypeInfo {
        package_name: Cow::Owned(package.clone()),
        message_name: Cow::Owned(format!("{action_type}_Goal")),
        messages: messages.clone(),
    };
    let result_info = TypeInfo {
        package_name: Cow::Owned(package.clone()),
        message_name: Cow::Owned(format!("{action_type}_Result")),
        messages: messages.clone(),
    };
    let feedback_info = TypeInfo {
        package_name: Cow::Owned(package),
        message_name: Cow::Owned(format!("{action_type}_Feedback")),
        messages,
    };
    let qos = config.qos.to_neutral_qos();
    let qos_string = ZenohQosMapping::from_ros_qos(&qos).to_string();
    let token =
        |name: &str,
         identity: &dora_ros2_bridge::transport::zenoh::compatibility::RosTypeIdentity| {
            TopicToken {
                name: name.into(),
                type_name: identity.dds_name.clone(),
                type_hash: identity.key_hash_component(),
                qos: qos_string.clone(),
            }
        };
    let tokens = ActionTokens {
        send_goal: token(&endpoints.send_goal.name, &send_goal_identity),
        get_result: token(&endpoints.get_result.name, &get_result_identity),
        cancel_goal: token(&endpoints.cancel_goal.name, &cancel),
        feedback: token(&endpoints.feedback.name, &feedback_identity),
        status: TopicToken {
            qos: ZenohQosMapping::from_ros_qos(
                &dora_ros2_bridge::transport::action::zenoh::action_status_qos(&qos),
            )
            .to_string(),
            ..token(&endpoints.status.name, &status)
        },
    };
    match config.role.as_ref().context("role required for action")? {
        Ros2Role::Client => {
            let readiness_tokens = tokens.clone();
            let client = futures::executor::block_on(ZenohActionClient::declare(
                zenoh_node,
                &keys,
                tokens,
                &qos,
                64 * 1024 * 1024,
            ))?;
            futures::executor::block_on(
                dora_ros2_bridge::transport::action::zenoh::wait_for_server(
                    zenoh_node.graph(),
                    &readiness_tokens,
                    std::time::Instant::now() + Duration::from_secs(20),
                ),
            )?;
            run_zenoh_action_client(client, goal_info, result_info, feedback_info)
        }
        Ros2Role::Server => run_zenoh_action_server(
            futures::executor::block_on(ZenohActionServer::declare(
                zenoh_node, &keys, tokens, &qos,
            ))?,
            goal_info,
            result_info,
            feedback_info,
        ),
    }
}

fn run_zenoh_action_client(
    client: ZenohActionClient,
    goal_type_info: TypeInfo<'static>,
    result_type_info: TypeInfo<'static>,
    feedback_type_info: TypeInfo<'static>,
) -> eyre::Result<()> {
    use ros2_client::action::{
        FeedbackMessage, GetResultRequest, GetResultResponse, SendGoalRequest, SendGoalResponse,
    };

    let (mut node, dora_events) = DoraNode::init_from_env()?;
    let client = Arc::new(client);
    let goal_limit = ConcurrentGoalLimit::new(MAX_CONCURRENT_GOALS);
    let (tx, rx) = flume::bounded::<ActionEvent>(MAX_CONCURRENT_GOALS * 2);
    let feedback_client = client.clone();
    let feedback_tx = tx.clone();
    std::thread::spawn(move || {
        let _guard = TypeInfoGuard::deserialize(feedback_type_info);
        futures::executor::block_on(async move {
            while let Ok((payload, _)) = feedback_client.feedback.recv_async().await {
                match deserialize_cdr::<FeedbackMessage<BridgeMessage>>(&payload) {
                    Ok(message) => {
                        if let Some(data) = message.feedback.0
                            && feedback_tx.send(ActionEvent::Feedback(data)).is_err()
                        {
                            break;
                        }
                    }
                    Err(error) => tracing::warn!("invalid Zenoh action feedback: {error}"),
                }
            }
        });
    });
    let merged = dora_events.merge_external(Box::pin(rx.into_stream()));
    for event in futures::executor::block_on_stream(merged) {
        match event {
            MergedEvent::Dora(Event::Input { data, .. }) => {
                let permit = match goal_limit.try_acquire() {
                    Ok(permit) => permit,
                    Err(error) => {
                        tracing::warn!("{error}");
                        continue;
                    }
                };
                let goal_id = ros2_client::action::GoalId::new_random();
                let request = {
                    let _guard = TypeInfoGuard::serialize(goal_type_info.clone());
                    serialize_cdr(&SendGoalRequest {
                        goal_id,
                        goal: BridgeMessage(Some(data.to_data())),
                    })?
                };
                let Some(response) = peer_value_or_warn(
                    futures::executor::block_on(
                        client.send_goal.call(request, ACTION_GOAL_TIMEOUT),
                    ),
                    "Zenoh action send-goal failed",
                ) else {
                    continue;
                };
                let Some(response): Option<SendGoalResponse> = peer_value_or_warn(
                    deserialize_cdr(&response),
                    "invalid Zenoh action send-goal response",
                ) else {
                    continue;
                };
                if !response.accepted {
                    tracing::warn!("action goal was rejected by server");
                    continue;
                }
                let result_client = client.clone();
                let result_tx = tx.clone();
                let result_info = result_type_info.clone();
                std::thread::spawn(move || {
                    let _permit = permit;
                    let request = match serialize_cdr(&GetResultRequest { goal_id }) {
                        Ok(request) => request,
                        Err(error) => {
                            tracing::warn!("failed to encode get-result request: {error}");
                            return;
                        }
                    };
                    let response = futures::executor::block_on(result_client.get_result.call(
                        request,
                        dora_ros2_bridge::transport::action::zenoh::GET_RESULT_TIMEOUT,
                    ));
                    let response = match response {
                        Ok(response) => response,
                        Err(error) => {
                            tracing::warn!("Zenoh action result error: {error}");
                            return;
                        }
                    };
                    let _guard = TypeInfoGuard::deserialize(result_info);
                    match deserialize_cdr::<GetResultResponse<BridgeMessage>>(&response) {
                        Ok(response) => {
                            if let Some(data) = response.result.0 {
                                let _ = result_tx.send(ActionEvent::Result(data));
                            }
                        }
                        Err(error) => tracing::warn!("invalid Zenoh action result: {error}"),
                    }
                });
            }
            MergedEvent::Dora(Event::Stop(_)) => break,
            MergedEvent::Dora(_) => {}
            MergedEvent::External(ActionEvent::Feedback(data)) => node.send_output(
                "feedback".into(),
                Default::default(),
                StructArray::from(data),
            )?,
            MergedEvent::External(ActionEvent::Result(data)) => {
                node.send_output("result".into(), Default::default(), StructArray::from(data))?
            }
        }
    }
    Ok(())
}

enum ZenohActionServerEvent {
    Goal(dora_ros2_bridge::transport::zenoh::service::ServiceRequest),
    ResultRequest(dora_ros2_bridge::transport::zenoh::service::ServiceRequest),
    Cancel(dora_ros2_bridge::transport::zenoh::service::ServiceRequest),
}

struct ZenohServerGoal {
    id: ros2_client::action::GoalId,
    status: ros2_client::action::GoalStatusEnum,
    result: Option<Vec<u8>>,
    result_request: Option<dora_ros2_bridge::transport::RequestId>,
}

#[derive(serde::Deserialize)]
struct CancelGoalRequestWire {
    goal_info: ros2_client::action::GoalInfo,
}

fn run_zenoh_action_server(
    server: ZenohActionServer,
    goal_type_info: TypeInfo<'static>,
    result_type_info: TypeInfo<'static>,
    feedback_type_info: TypeInfo<'static>,
) -> eyre::Result<()> {
    use ros2_client::action::{
        CancelGoalResponse, FeedbackMessage, GetResultRequest, GetResultResponse, GoalInfo,
        GoalStatusEnum, SendGoalRequest, SendGoalResponse,
    };

    let server = Arc::new(server);
    let (mut node, events) = DoraNode::init_from_env()?;
    let (tx, rx) = flume::bounded(MAX_CONCURRENT_GOALS * 4);
    for kind in 0..3 {
        let server = server.clone();
        let tx = tx.clone();
        std::thread::spawn(move || {
            futures::executor::block_on(async move {
                loop {
                    let request = match kind {
                        0 => server.send_goal.recv().await,
                        1 => server.get_result.recv().await,
                        _ => server.cancel_goal.recv().await,
                    };
                    let event = match (kind, request) {
                        (0, Ok(request)) => ZenohActionServerEvent::Goal(request),
                        (1, Ok(request)) => ZenohActionServerEvent::ResultRequest(request),
                        (_, Ok(request)) => ZenohActionServerEvent::Cancel(request),
                        (_, Err(_)) => break,
                    };
                    if tx.send(event).is_err() {
                        break;
                    }
                }
            });
        });
    }
    drop(tx);
    let mut goals: GoalSlots<String, ZenohServerGoal> = GoalSlots::new(MAX_CONCURRENT_GOALS);
    let merged = events.merge_external(Box::pin(rx.into_stream()));
    for event in futures::executor::block_on_stream(merged) {
        match event {
            MergedEvent::External(ZenohActionServerEvent::Goal(request)) => {
                let decoded = {
                    let _guard = TypeInfoGuard::deserialize(goal_type_info.clone());
                    deserialize_cdr::<SendGoalRequest<BridgeMessage>>(&request.payload)
                };
                let decoded = match decoded {
                    Ok(decoded) => decoded,
                    Err(error) => {
                        futures::executor::block_on(
                            server.send_goal.reject(request.id, &error.to_string()),
                        )?;
                        continue;
                    }
                };
                let key = decoded.goal_id.uuid.to_string();
                // Retire completed goals the client never polled so a burst of
                // finished goals can't permanently fill the slot table and reject
                // all future work. Executing/Canceling goals are always kept.
                if goals.len() >= MAX_CONCURRENT_GOALS {
                    let finished: Vec<String> = goals
                        .iter()
                        .filter(|(_, goal)| {
                            matches!(
                                goal.status,
                                GoalStatusEnum::Succeeded
                                    | GoalStatusEnum::Aborted
                                    | GoalStatusEnum::Canceled
                            )
                        })
                        .map(|(k, _)| k.clone())
                        .collect();
                    for k in finished {
                        goals.remove(&k);
                    }
                }
                let accepted = goals.len() < MAX_CONCURRENT_GOALS && decoded.goal.0.is_some();
                let response = serialize_cdr(&SendGoalResponse {
                    accepted,
                    stamp: ros2_client::builtin_interfaces::Time::from_nanos(unix_timestamp_ns()),
                })?;
                if let Err(error) =
                    futures::executor::block_on(server.send_goal.reply(request.id, &response))
                {
                    tracing::warn!("failed to reply to Zenoh send-goal request: {error}");
                }
                if accepted {
                    let data = decoded.goal.0.expect("accepted goal has payload");
                    goals.insert(
                        key.clone(),
                        ZenohServerGoal {
                            id: decoded.goal_id,
                            status: GoalStatusEnum::Executing,
                            result: None,
                            result_request: None,
                        },
                    )?;
                    let mut metadata = dora_message::metadata::MetadataParameters::default();
                    metadata.insert(GOAL_ID.into(), Parameter::String(key));
                    node.send_output("goal".into(), metadata, StructArray::from(data))?;
                    publish_zenoh_status(&server, &goals)?;
                }
            }
            MergedEvent::External(ZenohActionServerEvent::ResultRequest(request)) => {
                let Some(decoded): Option<GetResultRequest> = peer_value_or_warn(
                    deserialize_cdr(&request.payload),
                    "invalid Zenoh action get-result request",
                ) else {
                    futures::executor::block_on(
                        server
                            .get_result
                            .reject(request.id, "invalid request payload"),
                    )?;
                    continue;
                };
                let key = decoded.goal_id.uuid.to_string();
                let delivered = match goals.get_mut(&key) {
                    Some(goal) => match goal.result.as_ref() {
                        Some(response) => {
                            if let Err(error) = futures::executor::block_on(
                                server.get_result.reply(request.id, response),
                            ) {
                                tracing::warn!(
                                    "failed to reply to Zenoh get-result request: {error}"
                                );
                            }
                            true
                        }
                        None => {
                            goal.result_request = Some(request.id);
                            false
                        }
                    },
                    None => {
                        futures::executor::block_on(
                            server.get_result.reject(request.id, "unknown goal"),
                        )?;
                        false
                    }
                };
                if delivered {
                    // The client has its result; retire the goal so completed goals
                    // don't accumulate in the slot table or the published status.
                    goals.remove(&key);
                    publish_zenoh_status(&server, &goals)?;
                }
            }
            MergedEvent::External(ZenohActionServerEvent::Cancel(request)) => {
                let Some(decoded): Option<CancelGoalRequestWire> = peer_value_or_warn(
                    deserialize_cdr(&request.payload),
                    "invalid Zenoh action cancel-goal request",
                ) else {
                    futures::executor::block_on(
                        server
                            .cancel_goal
                            .reject(request.id, "invalid request payload"),
                    )?;
                    continue;
                };
                let requested = decoded.goal_info.goal_id.uuid;
                let mut canceling = Vec::new();
                for (_, goal) in goals.iter() {
                    if requested.is_nil() || requested == goal.id.uuid {
                        canceling.push(GoalInfo {
                            goal_id: goal.id,
                            stamp: ros2_client::builtin_interfaces::Time::ZERO,
                        });
                    }
                }
                for info in &canceling {
                    if let Some(goal) = goals.get_mut(&info.goal_id.uuid.to_string()) {
                        goal.status = GoalStatusEnum::Canceling;
                    }
                }
                let response = CancelGoalResponse {
                    return_code: if canceling.is_empty() {
                        ros2_client::action_msgs::CancelGoalResponseEnum::UnknownGoal
                    } else {
                        ros2_client::action_msgs::CancelGoalResponseEnum::None
                    },
                    goals_canceling: canceling,
                };
                let payload = serialize_cdr(&response)?;
                if let Err(error) =
                    futures::executor::block_on(server.cancel_goal.reply(request.id, &payload))
                {
                    tracing::warn!("failed to reply to Zenoh cancel-goal request: {error}");
                }
                publish_zenoh_status(&server, &goals)?;
            }
            MergedEvent::Dora(Event::Input { id, metadata, data }) => {
                let Some(goal_key) =
                    get_string_param(&metadata.parameters, GOAL_ID).map(str::to_owned)
                else {
                    tracing::warn!("action server input `{id}` missing goal_id metadata");
                    continue;
                };
                let Some(goal) = goals.get_mut(&goal_key) else {
                    tracing::warn!("action input for unknown goal {goal_key}");
                    continue;
                };
                match id.as_str() {
                    "feedback" => {
                        let payload = {
                            let _guard = TypeInfoGuard::serialize(feedback_type_info.clone());
                            serialize_cdr(&FeedbackMessage {
                                goal_id: goal.id,
                                feedback: BridgeMessage(Some(data.to_data())),
                            })?
                        };
                        futures::executor::block_on(server.feedback.publish(&payload))?;
                    }
                    "result" => {
                        goal.status = match get_string_param(&metadata.parameters, GOAL_STATUS) {
                            Some(GOAL_STATUS_CANCELED) => GoalStatusEnum::Canceled,
                            Some(GOAL_STATUS_ABORTED) => GoalStatusEnum::Aborted,
                            _ => GoalStatusEnum::Succeeded,
                        };
                        let response = {
                            let _guard = TypeInfoGuard::serialize(result_type_info.clone());
                            serialize_cdr(&GetResultResponse {
                                status: goal.status,
                                result: BridgeMessage(Some(data.to_data())),
                            })?
                        };
                        let delivered = if let Some(request_id) = goal.result_request.take() {
                            if let Err(error) = futures::executor::block_on(
                                server.get_result.reply(request_id, &response),
                            ) {
                                tracing::warn!(
                                    "failed to reply to Zenoh get-result request: {error}"
                                );
                            }
                            true
                        } else {
                            goal.result = Some(response);
                            false
                        };
                        // If a get-result request was already waiting, the result is
                        // delivered now and the goal is retired; otherwise keep it
                        // (with the stored result) until the client polls.
                        if delivered {
                            goals.remove(&goal_key);
                        }
                        publish_zenoh_status(&server, &goals)?;
                    }
                    other => tracing::warn!("unexpected action server input `{other}`"),
                }
            }
            MergedEvent::Dora(Event::Stop(_)) => break,
            MergedEvent::Dora(_) => {}
        }
    }
    Ok(())
}

fn publish_zenoh_status(
    server: &ZenohActionServer,
    goals: &GoalSlots<String, ZenohServerGoal>,
) -> eyre::Result<()> {
    let status = ros2_client::action_msgs::GoalStatusArray {
        status_list: goals
            .iter()
            .map(|(_, goal)| ros2_client::action_msgs::GoalStatus {
                goal_info: ros2_client::action::GoalInfo {
                    goal_id: goal.id,
                    stamp: ros2_client::builtin_interfaces::Time::ZERO,
                },
                status: goal.status,
            })
            .collect(),
    };
    futures::executor::block_on(server.status.publish(&serialize_cdr(&status)?))?;
    Ok(())
}

fn unix_timestamp_ns() -> i64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .ok()
        .and_then(|duration| i64::try_from(duration.as_nanos()).ok())
        .unwrap_or(0)
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

    let (node, dora_events) = DoraNode::init_from_env()?;

    if subscribers.is_empty() {
        run_publish_only(node, dora_events, publishers, &messages)?;
    } else if subscribers.len() == 1 {
        let (output_id, sub) = subscribers.into_iter().next().unwrap();
        run_single_subscriber(node, dora_events, output_id, sub, publishers, &messages)?;
    } else {
        run_multi_subscriber(node, dora_events, subscribers, publishers, &messages)?;
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
    let service_name = config
        .service
        .as_ref()
        .context("service name required for service mode")?;
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
                    dora_ros2_bridge::detect_service_mapping(),
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
                    dora_ros2_bridge::detect_service_mapping(),
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
    let (mut node, dora_events) = DoraNode::init_from_env()?;

    for event in futures::executor::block_on_stream(dora_events) {
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
    let (mut node, dora_events) = DoraNode::init_from_env()?;

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
    let merged = dora_events.merge_external(Box::pin(request_stream));

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
            MergedEvent::Dora(Event::Input {
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
            MergedEvent::Dora(Event::Stop(_)) => break,
            MergedEvent::Dora(_) => {}
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
                let request_id = DoraNode::new_request_id();
                if oldest_insert.is_none() {
                    oldest_insert = Some(now);
                }
                pending_requests.insert(request_id.clone(), (rmw_id, now));
                let mut params = dora_message::metadata::MetadataParameters::default();
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
    let action_name = config
        .action
        .as_ref()
        .context("action name required for action mode")?;
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
                    dora_ros2_bridge::detect_service_mapping(),
                    &action_ros2_name,
                    &action_type_name,
                    action_qos,
                )
                .map_err(|e| eyre!("failed to create action client: {e:?}"))?;

            // NOTE: ros2_client does not provide a wait_for_action_server API.
            // The first goal send will time out (ACTION_GOAL_TIMEOUT) if the
            // action server is not yet available. Start the action server
            // before this dataflow for reliable operation.
            tracing::info!("action client created, waiting for goals from Dora inputs");

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
                    dora_ros2_bridge::detect_service_mapping(),
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
    let (mut node, dora_events) = DoraNode::init_from_env()?;
    let client = Arc::new(client);
    let goal_limit = ConcurrentGoalLimit::new(MAX_CONCURRENT_GOALS);

    // Channel for feedback and result from background threads.
    // Bounded at 16 per concurrent goal to provide backpressure — if the main
    // loop can't keep up, feedback threads will block rather than consume
    // unbounded memory. `tx` is cloned per-goal; when the loop breaks on Stop,
    // background threads detect the closed channel and exit.
    let (tx, rx) = flume::bounded::<ActionEvent>(MAX_CONCURRENT_GOALS * 2);

    let rx_stream = rx.into_stream();
    let merged = dora_events.merge_external(Box::pin(rx_stream));

    for event in futures::executor::block_on_stream(merged) {
        match event {
            MergedEvent::Dora(Event::Input {
                id: _,
                metadata: _,
                data,
            }) => {
                // Cap concurrent in-flight goals
                let permit = match goal_limit.try_acquire() {
                    Ok(permit) => permit,
                    Err(_) => {
                        tracing::warn!(
                            "max concurrent goals ({MAX_CONCURRENT_GOALS}) reached, dropping goal"
                        );
                        continue;
                    }
                };

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
                std::thread::spawn(move || {
                    let _permit = permit;
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
                });
            }
            MergedEvent::Dora(Event::Stop(_)) => break,
            MergedEvent::Dora(_) => {}
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
    let (mut node, dora_events) = DoraNode::init_from_env()?;
    let server = Arc::new(server);

    // Map goal_id -> ExecutingGoalHandle for dispatching feedback/result
    let mut executing_goals: GoalSlots<
        String,
        ros2_client::action::ExecutingGoalHandle<BridgeMessage>,
    > = GoalSlots::new(MAX_CONCURRENT_GOALS);

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
    let merged = dora_events.merge_external(Box::pin(rx_stream));

    for event in futures::executor::block_on_stream(merged) {
        match event {
            MergedEvent::Dora(Event::Input { id, metadata, data }) => {
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
            MergedEvent::Dora(Event::Stop(_)) => break,
            MergedEvent::Dora(_) => {}
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
                    executing_goals.insert(goal_id, handle)?;
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
    _node: DoraNode,
    dora_events: dora_node_api::EventStream,
    publishers: Vec<(
        String,
        TypeInfo<'static>,
        ros2_client::Publisher<TypedValue<'static>>,
    )>,
    messages: &Arc<HashMap<String, HashMap<String, Message>>>,
) -> eyre::Result<()> {
    for event in futures::executor::block_on_stream(dora_events) {
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
    mut node: DoraNode,
    dora_events: dora_node_api::EventStream,
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
    let merged = dora_events.merge_external(Box::pin(ros_stream));

    for event in futures::executor::block_on_stream(merged) {
        match event {
            MergedEvent::Dora(Event::Input {
                id,
                metadata: _,
                data,
            }) => {
                handle_publish_input(&id, &data, &publishers, messages)?;
            }
            MergedEvent::Dora(Event::Stop(_)) => break,
            MergedEvent::Dora(_) => {}
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
    mut node: DoraNode,
    dora_events: dora_node_api::EventStream,
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
    let merged = dora_events.merge_external(Box::pin(rx_stream));

    for event in futures::executor::block_on_stream(merged) {
        match event {
            MergedEvent::Dora(Event::Input {
                id,
                metadata: _,
                data,
            }) => {
                handle_publish_input(&id, &data, &publishers, messages)?;
            }
            MergedEvent::Dora(Event::Stop(_)) => break,
            MergedEvent::Dora(_) => {}
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
        .unwrap_or_else(|| "dora_ros2_bridge".to_string());
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
/// NOTE: This blocks the main thread and does not respond to Dora Stop events.
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

    let packages = dora_ros2_bridge_msg_gen::get_packages(&path_refs)
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

trait ToNeutralQos {
    fn to_neutral_qos(&self) -> Ros2Qos;
    fn to_rustdds_qos(&self) -> rustdds::QosPolicies {
        dora_ros2_bridge::transport::dds::to_rustdds_qos(&self.to_neutral_qos())
    }
}

impl ToNeutralQos for Ros2QosConfig {
    fn to_neutral_qos(&self) -> Ros2Qos {
        let lease_duration = self.lease_duration.map(Duration::from_secs_f64);
        Ros2Qos {
            reliability: if self.reliable {
                Reliability::Reliable {
                    max_blocking_time: self
                        .max_blocking_time
                        .map(Duration::from_secs_f64)
                        .unwrap_or(Duration::from_millis(100)),
                }
            } else {
                Reliability::BestEffort
            },
            durability: if self.durability.as_deref() == Some("transient_local") {
                Durability::TransientLocal
            } else {
                Durability::Volatile
            },
            history: if self.keep_all {
                History::KeepAll
            } else {
                History::KeepLast {
                    depth: self.keep_last.unwrap_or(1).max(1),
                }
            },
            liveliness: match self.liveliness.as_deref() {
                Some("manual_by_participant") => Liveliness::ManualByParticipant { lease_duration },
                Some("manual_by_topic") => Liveliness::ManualByTopic { lease_duration },
                _ => Liveliness::Automatic { lease_duration },
            },
        }
    }
}

#[cfg(test)]
mod peer_failure_tests {
    use super::peer_value_or_warn;

    #[test]
    fn malformed_peer_value_is_dropped_without_poisoning_the_next_value() {
        let malformed: eyre::Result<u32> = Err(eyre::eyre!("malformed"));
        assert!(peer_value_or_warn(malformed, "test").is_none());
        let valid: eyre::Result<u32> = Ok(42);
        assert_eq!(peer_value_or_warn(valid, "test"), Some(42));
    }
}
