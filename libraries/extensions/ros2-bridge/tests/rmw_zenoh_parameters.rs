#![cfg(feature = "rmw-zenoh")]

use std::{sync::Arc, time::Duration};

use dora_message::descriptor::RmwZenohCompatibility;
use dora_ros2_bridge::{
    ros2_client::{
        NodeName, ParameterValue,
        parameters::raw,
        rcl_interfaces::{
            DescribeParametersRequest, DescribeParametersResponse, GetParameterTypesRequest,
            GetParameterTypesResponse, GetParametersRequest, GetParametersResponse,
            ListParametersRequest, ListParametersResponse, SetParametersRequest,
            SetParametersResponse,
        },
    },
    transport::{
        Ros2Qos,
        zenoh::{
            Context, ContextOptions, Node,
            compatibility::{TypeDescriptionResolver, resolve_message, resolve_service},
            deserialize_cdr,
            graph::GraphSnapshot,
            keyexpr::{DataKey, EntityKind, TopicToken},
            parameters::{ParameterServer, ParameterTypes},
            pubsub::NodeSubscription,
            serialize_cdr,
            service::NodeServiceClient,
        },
    },
};
use serde::{Deserialize, Serialize, de::DeserializeOwned};

const NODE: &str = "/robot/arm";

fn resolver() -> TypeDescriptionResolver {
    TypeDescriptionResolver::from_ament_prefix_path()
}

/// Calls `<NODE>/<service>` the way a ROS client does: CDR request in, CDR
/// response out.
async fn call<Req: Serialize, Res: DeserializeOwned>(
    node: &Node,
    service: &str,
    type_name: &str,
    request: &Req,
) -> Res {
    call_on(node, NODE, service, type_name, request).await
}

/// Calls `<node_name>/<service>`.
async fn call_on<Req: Serialize, Res: DeserializeOwned>(
    node: &Node,
    node_name: &str,
    service: &str,
    type_name: &str,
    request: &Req,
) -> Res {
    let identity = resolve_service(
        RmwZenohCompatibility::Humble,
        "rcl_interfaces",
        type_name,
        &resolver(),
    )
    .unwrap();
    let name = format!("{node_name}/{service}");
    let key = DataKey::new(node.domain(), &name, &identity).unwrap();
    let token = TopicToken {
        name,
        type_name: identity.dds_name.clone(),
        type_hash: identity.key_hash_component(),
        qos: "::,1000:,:,:,,".into(),
    };
    let client = NodeServiceClient::declare(node, key.as_str(), token, 4)
        .await
        .unwrap();
    let response = client
        .call(serialize_cdr(request).unwrap(), Duration::from_secs(5))
        .await
        .unwrap();
    deserialize_cdr(&response).unwrap()
}

fn parameter(name: &str, value: ParameterValue) -> raw::Parameter {
    raw::Parameter {
        name: name.into(),
        value: value.into(),
    }
}

/// `rcl_interfaces/msg/ParameterEvent` as ROS sends it, with a
/// `builtin_interfaces/Time` stamp.
#[derive(Deserialize)]
struct ParameterEvent {
    stamp: (i32, u32),
    node: String,
    new_parameters: Vec<raw::Parameter>,
    changed_parameters: Vec<raw::Parameter>,
    deleted_parameters: Vec<raw::Parameter>,
}

#[derive(Deserialize)]
struct SetParametersAtomicallyResponse {
    result: raw::SetParametersResult,
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn parameter_services_answer_like_ros2_client() {
    let context = Context::open(ContextOptions {
        domain_id: 44,
        config_uri: None,
    })
    .await
    .unwrap();
    let node = context
        .create_node("zid", "nid", "/", "/robot", "arm")
        .await
        .unwrap();
    let event_type = resolve_message(
        RmwZenohCompatibility::Humble,
        "rcl_interfaces",
        "ParameterEvent",
        &resolver(),
    )
    .unwrap();
    let events = NodeSubscription::declare(
        &node,
        DataKey::new(node.domain(), "/parameter_events", &event_type)
            .unwrap()
            .as_str(),
        TopicToken {
            name: "/parameter_events".into(),
            type_name: event_type.dds_name.clone(),
            type_hash: event_type.key_hash_component(),
            qos: "::,1000:,:,:,,".into(),
        },
        &Ros2Qos::default(),
        8,
        64 * 1024,
        Arc::new(|bytes: &[u8]| Ok(bytes.to_vec())),
    )
    .await
    .unwrap();

    let types = ParameterTypes::resolve(RmwZenohCompatibility::Humble, &resolver()).unwrap();
    let server = ParameterServer::declare(
        &node,
        &NodeName::new("/robot", "arm").unwrap(),
        &types,
        vec![("speed".into(), ParameterValue::Double(1.5))],
        Some(Box::new(|name, value| {
            if name == "speed" && !matches!(value, ParameterValue::Double(_)) {
                Err("speed must stay a double".into())
            } else {
                Ok(())
            }
        })),
    )
    .await
    .unwrap();

    // All six services and the event publisher are in the graph, under the node.
    let services: Vec<_> = [
        "get_parameters",
        "get_parameter_types",
        "set_parameters",
        "set_parameters_atomically",
        "list_parameters",
        "describe_parameters",
    ]
    .iter()
    .map(|service| format!("{NODE}/{service}"))
    .collect();
    wait_for_graph(&context, |snapshot| {
        let names = |kind| {
            snapshot
                .entities
                .iter()
                .filter(move |entity| entity.token.kind == kind && entity.token.node_name == "arm")
                .filter_map(|entity| entity.token.topic.as_ref().map(|topic| topic.name.clone()))
                .collect::<Vec<_>>()
        };
        services
            .iter()
            .all(|service| names(EntityKind::Service).contains(service))
            && names(EntityKind::Publisher).contains(&"/parameter_events".to_string())
    })
    .await;

    let get: GetParametersResponse = call(
        &node,
        "get_parameters",
        "GetParameters",
        &GetParametersRequest {
            names: vec!["speed".into(), "missing".into()],
        },
    )
    .await;
    assert_eq!(get.values.len(), 2);
    assert_eq!(get.values[0].ptype, raw::ParameterType::DOUBLE);
    assert_eq!(get.values[0].double_value, 1.5);
    assert_eq!(get.values[1].ptype, raw::ParameterType::NOT_SET);

    let types: GetParameterTypesResponse = call(
        &node,
        "get_parameter_types",
        "GetParameterTypes",
        &GetParameterTypesRequest {
            names: vec!["speed".into(), "use_sim_time".into(), "missing".into()],
        },
    )
    .await;
    assert_eq!(
        types.values,
        [
            raw::ParameterType::DOUBLE,
            raw::ParameterType::BOOL,
            raw::ParameterType::NOT_SET
        ]
    );

    let set: SetParametersResponse = call(
        &node,
        "set_parameters",
        "SetParameters",
        &SetParametersRequest {
            parameter: vec![
                parameter("speed", ParameterValue::Double(2.0)),
                parameter("speed", ParameterValue::String("fast".into())),
                parameter("missing", ParameterValue::Integer(1)),
                parameter("use_sim_time", ParameterValue::Integer(1)),
            ],
        },
    )
    .await;
    let results: Vec<_> = set.results.iter().map(|result| result.successful).collect();
    assert_eq!(results, [true, false, false, false]);
    assert_eq!(set.results[1].reason, "speed must stay a double");
    assert_eq!(
        set.results[2].reason,
        "Setting undeclared parameter 'missing' is not allowed."
    );
    assert_eq!(server.get("speed"), Some(ParameterValue::Double(2.0)));

    // The accepted set is published; the rejected ones aren't.
    let event = tokio::time::timeout(Duration::from_secs(5), events.recv_async())
        .await
        .unwrap()
        .unwrap();
    let event: ParameterEvent = deserialize_cdr(&event.0).unwrap();
    assert_eq!(event.node, NODE);
    assert!(event.stamp.0 > 0);
    assert!(event.new_parameters.is_empty() && event.deleted_parameters.is_empty());
    assert_eq!(event.changed_parameters.len(), 1);
    assert_eq!(event.changed_parameters[0].name, "speed");
    assert_eq!(event.changed_parameters[0].value.double_value, 2.0);

    // Atomically: one bad value and nothing is set.
    let atomic: SetParametersAtomicallyResponse = call(
        &node,
        "set_parameters_atomically",
        "SetParametersAtomically",
        &SetParametersRequest {
            parameter: vec![
                parameter("speed", ParameterValue::Double(3.0)),
                parameter("missing", ParameterValue::Integer(1)),
            ],
        },
    )
    .await;
    assert!(!atomic.result.successful);
    assert_eq!(server.get("speed"), Some(ParameterValue::Double(2.0)));
    let atomic: SetParametersAtomicallyResponse = call(
        &node,
        "set_parameters_atomically",
        "SetParametersAtomically",
        &SetParametersRequest {
            parameter: vec![
                parameter("speed", ParameterValue::Double(3.0)),
                parameter("use_sim_time", ParameterValue::Boolean(true)),
            ],
        },
    )
    .await;
    assert!(atomic.result.successful);
    assert_eq!(server.get("speed"), Some(ParameterValue::Double(3.0)));
    assert_eq!(
        server.get("use_sim_time"),
        Some(ParameterValue::Boolean(true))
    );

    let list: ListParametersResponse = call(
        &node,
        "list_parameters",
        "ListParameters",
        &ListParametersRequest {
            prefixes: vec![],
            depth: 0,
        },
    )
    .await;
    assert_eq!(list.result.names, ["speed", "use_sim_time"]);
    let list: ListParametersResponse = call(
        &node,
        "list_parameters",
        "ListParameters",
        &ListParametersRequest {
            prefixes: vec!["use_".into()],
            depth: 0,
        },
    )
    .await;
    assert_eq!(list.result.names, ["use_sim_time"]);

    let described: DescribeParametersResponse = call(
        &node,
        "describe_parameters",
        "DescribeParameters",
        &DescribeParametersRequest {
            names: vec!["speed".into(), "missing".into()],
        },
    )
    .await;
    assert_eq!(described.values[0].name, "speed");
    assert_eq!(described.values[0].r#type, raw::ParameterType::DOUBLE);
    assert_eq!(described.values[1].r#type, raw::ParameterType::NOT_SET);

    // A local set goes through the same checks and is published too.
    assert!(
        server
            .set("speed", ParameterValue::Integer(4))
            .await
            .is_err()
    );
    server
        .set("speed", ParameterValue::Double(4.0))
        .await
        .unwrap();
    assert_eq!(next_speed(&events, 4.0).await, 4.0);

    // Dropping the server takes its services out of the graph.
    drop(server);
    wait_for_graph(&context, |snapshot| {
        !snapshot.entities.iter().any(|entity| {
            entity.token.kind == EntityKind::Service && entity.token.node_name == "arm"
        })
    })
    .await;
}

/// Waits for the event that sets `speed` to `expected`, skipping earlier ones.
async fn next_speed(events: &NodeSubscription<Vec<u8>>, expected: f64) -> f64 {
    loop {
        let event = tokio::time::timeout(Duration::from_secs(5), events.recv_async())
            .await
            .unwrap()
            .unwrap();
        let event: ParameterEvent = deserialize_cdr(&event.0).unwrap();
        if let Some(speed) = event
            .changed_parameters
            .iter()
            .find(|parameter| parameter.name == "speed")
            .filter(|parameter| parameter.value.double_value == expected)
        {
            return speed.value.double_value;
        }
    }
}

async fn wait_for_graph(context: &Context, predicate: impl Fn(&GraphSnapshot) -> bool) {
    tokio::time::timeout(Duration::from_secs(10), async {
        loop {
            let snapshot = context.graph.snapshot();
            if predicate(&snapshot) {
                break;
            }
            context
                .graph
                .wait_for_change(snapshot.generation)
                .await
                .expect("graph closed while waiting for a change");
        }
    })
    .await
    .expect("timed out waiting for the graph to reach the expected state");
}

/// Sets that race still publish their events in the order they were stored, a
/// panicking validator rejects the set instead of stopping the services, and an
/// empty set publishes nothing.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn concurrent_sets_publish_in_store_order() {
    let context = Context::open(ContextOptions {
        domain_id: 45,
        config_uri: None,
    })
    .await
    .unwrap();
    let node = context
        .create_node("zid", "nid", "/", "/robot", "race")
        .await
        .unwrap();
    let event_type = resolve_message(
        RmwZenohCompatibility::Humble,
        "rcl_interfaces",
        "ParameterEvent",
        &resolver(),
    )
    .unwrap();
    let events = NodeSubscription::declare(
        &node,
        DataKey::new(node.domain(), "/parameter_events", &event_type)
            .unwrap()
            .as_str(),
        TopicToken {
            name: "/parameter_events".into(),
            type_name: event_type.dds_name.clone(),
            type_hash: event_type.key_hash_component(),
            qos: "::,1000:,:,:,,".into(),
        },
        &Ros2Qos::default(),
        4096,
        64 * 1024,
        Arc::new(|bytes: &[u8]| Ok(bytes.to_vec())),
    )
    .await
    .unwrap();
    let types = ParameterTypes::resolve(RmwZenohCompatibility::Humble, &resolver()).unwrap();
    let server = Arc::new(
        ParameterServer::declare(
            &node,
            &NodeName::new("/robot", "race").unwrap(),
            &types,
            vec![("speed".into(), ParameterValue::Double(0.0))],
            Some(Box::new(|_, value| {
                if value == &ParameterValue::Double(-1.0) {
                    panic!("validator bug");
                }
                Ok(())
            })),
        )
        .await
        .unwrap(),
    );

    for round in 0..20 {
        let tasks: Vec<_> = (0..8)
            .map(|task| {
                let server = server.clone();
                tokio::spawn(async move {
                    let value = f64::from(round * 8 + task);
                    server
                        .set("speed", ParameterValue::Double(value))
                        .await
                        .unwrap();
                })
            })
            .collect();
        for task in tasks {
            task.await.unwrap();
        }
        let mut last = None;
        for _ in 0..8 {
            let event = tokio::time::timeout(Duration::from_secs(5), events.recv_async())
                .await
                .unwrap()
                .unwrap();
            let event: ParameterEvent = deserialize_cdr(&event.0).unwrap();
            assert_eq!(event.node, "/robot/race");
            last = Some(event.changed_parameters[0].value.double_value);
        }
        let Some(ParameterValue::Double(stored)) = server.get("speed") else {
            panic!("speed is not a double");
        };
        assert_eq!(last, Some(stored), "round {round}");
    }

    // A panicking validator rejects the value; the services keep answering.
    let error = server
        .set("speed", ParameterValue::Double(-1.0))
        .await
        .unwrap_err();
    assert!(error.contains("validator"), "{error}");
    let get: GetParametersResponse = call_on(
        &node,
        "/robot/race",
        "get_parameters",
        "GetParameters",
        &GetParametersRequest {
            names: vec!["speed".into()],
        },
    )
    .await;
    assert_eq!(get.values.len(), 1);

    // An empty atomic set changes nothing and publishes no event.
    let atomic: SetParametersAtomicallyResponse = call_on(
        &node,
        "/robot/race",
        "set_parameters_atomically",
        "SetParametersAtomically",
        &SetParametersRequest { parameter: vec![] },
    )
    .await;
    assert!(atomic.result.successful);
    assert!(
        tokio::time::timeout(Duration::from_millis(300), events.recv_async())
            .await
            .is_err(),
        "an empty set published an event"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn declare_forces_use_sim_time_and_queues_a_burst() {
    let context = Context::open(ContextOptions {
        domain_id: 46,
        config_uri: None,
    })
    .await
    .unwrap();
    let node = context
        .create_node("zid", "nid", "/", "/robot", "burst")
        .await
        .unwrap();
    let types = ParameterTypes::resolve(RmwZenohCompatibility::Humble, &resolver()).unwrap();
    // ros2-client always declares `use_sim_time` as `false`, replacing a
    // declared value.
    let _server = ParameterServer::declare(
        &node,
        &NodeName::new("/robot", "burst").unwrap(),
        &types,
        vec![("use_sim_time".into(), ParameterValue::Double(1.0))],
        None,
    )
    .await
    .unwrap();
    let get: GetParametersResponse = call_on(
        &node,
        "/robot/burst",
        "get_parameters",
        "GetParameters",
        &GetParametersRequest {
            names: vec!["use_sim_time".into()],
        },
    )
    .await;
    assert_eq!(
        get.values
            .into_iter()
            .map(ParameterValue::from)
            .collect::<Vec<_>>(),
        [ParameterValue::Boolean(false)]
    );

    // The services advertise a history depth of 1000, so a burst of 500
    // requests is queued, not dropped.
    let identity = resolve_service(
        RmwZenohCompatibility::Humble,
        "rcl_interfaces",
        "GetParameters",
        &resolver(),
    )
    .unwrap();
    let name = "/robot/burst/get_parameters".to_owned();
    let key = DataKey::new(node.domain(), &name, &identity).unwrap();
    let client = Arc::new(
        NodeServiceClient::declare(
            &node,
            key.as_str(),
            TopicToken {
                name,
                type_name: identity.dds_name.clone(),
                type_hash: identity.key_hash_component(),
                qos: "::,1000:,:,:,,".into(),
            },
            1024,
        )
        .await
        .unwrap(),
    );
    let request = serialize_cdr(&GetParametersRequest {
        names: vec!["use_sim_time".into()],
    })
    .unwrap();
    let calls: Vec<_> = (0..500)
        .map(|_| {
            let client = client.clone();
            let request = request.clone();
            tokio::spawn(async move { client.call(request, Duration::from_secs(5)).await })
        })
        .collect();
    for call in calls {
        call.await.unwrap().unwrap();
    }
}
