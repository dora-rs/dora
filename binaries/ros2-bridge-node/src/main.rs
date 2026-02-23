use std::{borrow::Cow, collections::HashMap, path::Path, sync::Arc};

use adora_message::descriptor::{Ros2BridgeConfig, Ros2Direction, Ros2QosConfig, Ros2TopicConfig};
use adora_node_api::{
    AdoraNode, Event,
    merged::{MergeExternal, MergedEvent},
};
use adora_ros2_bridge::{ros2_client, rustdds};
use adora_ros2_bridge_arrow::{TypeInfo, TypedValue, deserialize::StructDeserializer};
use adora_ros2_bridge_msg_gen::types::Message;
use arrow::array::{ArrayData, StructArray};
use eyre::{Context, ContextCompat, eyre};
use futures::{StreamExt, task::SpawnExt};

fn main() -> eyre::Result<()> {
    tracing_subscriber::fmt::init();

    let config_json = std::env::var("ADORA_ROS2_BRIDGE_CONFIG")
        .context("missing ADORA_ROS2_BRIDGE_CONFIG env var")?;
    let config: Ros2BridgeConfig =
        serde_json::from_str(&config_json).context("failed to parse ROS2 bridge config")?;

    let messages = load_messages()?;

    // Resolve single-topic vs multi-topic mode
    let topics = resolve_topics(&config)?;

    // Create ROS2 context and node
    let ros_context =
        ros2_client::Context::new().map_err(|e| eyre!("failed to create ROS2 context: {e:?}"))?;
    let node_name = config
        .node_name
        .unwrap_or_else(|| "adora_ros2_bridge".to_string());
    let namespace = config.namespace;
    let mut ros_node = ros_context
        .new_node(
            ros2_client::NodeName::new(&namespace, &node_name)
                .map_err(|e| eyre!("failed to create ROS2 node name: {e}"))?,
            ros2_client::NodeOptions::new().enable_rosout(true),
        )
        .map_err(|e| eyre!("failed to create ROS2 node: {e:?}"))?;

    // Start spinner for DDS discovery
    let pool = futures::executor::ThreadPool::new()?;
    let spinner = ros_node
        .spinner()
        .map_err(|e| eyre!("failed to create spinner: {e:?}"))?;
    pool.spawn(async {
        if let Err(err) = spinner.spin().await {
            eprintln!("ros2 spinner failed: {err:?}");
        }
    })
    .context("failed to spawn ros2 spinner")?;

    // Create subscriptions and publishers based on topic configs
    let mut subscribers: Vec<(String, SubscriptionStream)> = Vec::new();
    let mut publishers: Vec<(
        String,
        TypeInfo<'static>,
        ros2_client::Publisher<TypedValue<'static>>,
    )> = Vec::new();

    for topic_config in &topics {
        let (package, msg_name) = parse_message_type(&topic_config.message_type)?;
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

    // Initialize Adora node
    let (node, adora_events) = AdoraNode::init_from_env()?;

    // Run event loop
    if subscribers.is_empty() {
        // Publish-only mode: just process Adora events
        run_publish_only(node, adora_events, publishers, &messages)?;
    } else if subscribers.len() == 1 {
        // Single subscriber: merge its stream with Adora events
        let (output_id, sub) = subscribers.into_iter().next().unwrap();
        run_single_subscriber(node, adora_events, output_id, sub, publishers, &messages)?;
    } else {
        // Multiple subscribers: use channel-based merging
        run_multi_subscriber(node, adora_events, subscribers, publishers, &messages)?;
    }

    Ok(())
}

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
    // Use a channel to merge multiple subscription streams.
    // Each subscription runs on its own OS thread because async_stream_seed
    // borrows &self, so the subscription must outlive the stream.
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
    drop(tx); // drop the original sender so rx completes when all subs are done

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
        let _ = handle.join();
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

struct SubscriptionStream {
    deserializer: StructDeserializer<'static>,
    subscription: ros2_client::Subscription<ArrayData>,
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
        for message in package.messages {
            package_messages.insert(message.name.clone(), message);
        }
        messages.insert(package.name, package_messages);
    }

    if messages.is_empty() {
        tracing::warn!(
            "no ROS2 message definitions found. \
             Ensure AMENT_PREFIX_PATH is set and points to a sourced ROS2 workspace."
        );
    }

    Ok(Arc::new(messages))
}

fn parse_message_type(message_type: &str) -> eyre::Result<(String, String)> {
    let parts: Vec<&str> = message_type.split('/').collect();
    if parts.len() != 2 {
        eyre::bail!("invalid message_type format: {message_type}, expected package/MessageName");
    }
    Ok((parts[0].to_string(), parts[1].to_string()))
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
        _ => eyre::bail!("exactly one of `topic` or `topics` must be set"),
    }
}

trait ToRustddsQos {
    fn to_rustdds_qos(&self) -> rustdds::QosPolicies;
}

impl ToRustddsQos for Ros2QosConfig {
    fn to_rustdds_qos(&self) -> rustdds::QosPolicies {
        let mut builder = rustdds::QosPolicyBuilder::new();

        // Durability
        let durability = match self.durability.as_deref() {
            Some("transient_local") => rustdds::policy::Durability::TransientLocal,
            _ => rustdds::policy::Durability::Volatile,
        };
        builder = builder.durability(durability);

        // Reliability
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

        // History
        if self.keep_all {
            builder = builder.history(rustdds::policy::History::KeepAll);
        } else {
            let depth = self.keep_last.unwrap_or(1);
            builder = builder.history(rustdds::policy::History::KeepLast {
                depth: depth.max(1),
            });
        }

        // Liveliness
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
