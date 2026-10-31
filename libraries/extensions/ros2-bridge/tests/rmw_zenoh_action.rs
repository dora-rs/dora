#![cfg(feature = "rmw-zenoh")]

use dora_ros2_bridge::transport::action::{
    ActionEndpoints, ActionError, ActionState, ConcurrentGoalLimit, GoalSlots, GoalStatus,
    ResultAvailability,
};

#[test]
fn expands_the_five_ros_action_endpoints_exactly() {
    assert_eq!(
        dora_ros2_bridge_msg_gen::types::action_dds_type_names("example_interfaces", "Fibonacci"),
        [
            "example_interfaces::action::dds_::Fibonacci_SendGoal_",
            "example_interfaces::action::dds_::Fibonacci_GetResult_",
            "action_msgs::srv::dds_::CancelGoal_",
            "example_interfaces::action::dds_::Fibonacci_FeedbackMessage_",
            "action_msgs::msg::dds_::GoalStatusArray_",
        ]
    );
    let endpoints = ActionEndpoints::new("/fibonacci", "example_interfaces", "Fibonacci");
    assert_eq!(endpoints.send_goal.name, "/fibonacci/_action/send_goal");
    assert_eq!(
        endpoints.send_goal.type_name,
        "example_interfaces::action::dds_::Fibonacci_SendGoal_"
    );
    assert_eq!(endpoints.get_result.name, "/fibonacci/_action/get_result");
    assert_eq!(
        endpoints.get_result.type_name,
        "example_interfaces::action::dds_::Fibonacci_GetResult_"
    );
    assert_eq!(endpoints.cancel_goal.name, "/fibonacci/_action/cancel_goal");
    assert_eq!(
        endpoints.cancel_goal.type_name,
        "action_msgs::srv::dds_::CancelGoal_"
    );
    assert_eq!(endpoints.feedback.name, "/fibonacci/_action/feedback");
    assert_eq!(
        endpoints.feedback.type_name,
        "example_interfaces::action::dds_::Fibonacci_FeedbackMessage_"
    );
    assert_eq!(endpoints.status.name, "/fibonacci/_action/status");
    assert_eq!(
        endpoints.status.type_name,
        "action_msgs::msg::dds_::GoalStatusArray_"
    );
}

/// Same names as rmw_zenoh: the three services and two topics under
/// `<action>/_action/`, the status topic with its fixed QoS.
#[test]
fn resolves_the_rmw_zenoh_keys_and_tokens_of_an_action() {
    use dora_ros2_bridge::{
        dora_message::descriptor::RmwZenohCompatibility,
        transport::{
            Ros2Qos,
            action::zenoh::{action_status_qos, resolve_action_endpoints},
            zenoh::{compatibility::TypeDescriptionResolver, qos::ZenohQosMapping},
        },
    };

    let qos = Ros2Qos::default();
    let (keys, tokens) = resolve_action_endpoints(
        3,
        RmwZenohCompatibility::Humble,
        "/robot/fibonacci",
        "example_interfaces",
        "Fibonacci",
        &qos,
        &TypeDescriptionResolver::from_prefixes([]),
    )
    .unwrap();
    let key = |endpoint: &str, type_name: &str| {
        format!("3/robot/fibonacci/_action/{endpoint}/{type_name}/TypeHashNotSupported")
    };
    assert_eq!(
        keys.send_goal,
        key(
            "send_goal",
            "example_interfaces::action::dds_::Fibonacci_SendGoal_"
        )
    );
    assert_eq!(
        keys.get_result,
        key(
            "get_result",
            "example_interfaces::action::dds_::Fibonacci_GetResult_"
        )
    );
    assert_eq!(
        keys.cancel_goal,
        key("cancel_goal", "action_msgs::srv::dds_::CancelGoal_")
    );
    assert_eq!(
        keys.feedback,
        key(
            "feedback",
            "example_interfaces::action::dds_::Fibonacci_FeedbackMessage_"
        )
    );
    assert_eq!(
        keys.status,
        key("status", "action_msgs::msg::dds_::GoalStatusArray_")
    );

    assert_eq!(tokens.send_goal.name, "/robot/fibonacci/_action/send_goal");
    assert_eq!(
        tokens.cancel_goal.type_name,
        "action_msgs::srv::dds_::CancelGoal_"
    );
    assert_eq!(tokens.feedback.type_hash, "TypeHashNotSupported");
    let qos_string = ZenohQosMapping::from_ros_qos(&qos).to_string();
    assert_eq!(tokens.feedback.qos, qos_string);
    assert_eq!(
        tokens.status.qos,
        ZenohQosMapping::from_ros_qos(&action_status_qos(&qos)).to_string()
    );
    assert_ne!(tokens.status.qos, qos_string);
}

/// With REP-2016 type hashes (Jazzy and later), every endpoint carries the
/// hash of its own type, taken from the type descriptions of a real ROS
/// install (`msg-gen/test_type_descriptions`).
#[test]
fn rep2016_action_endpoints_carry_the_real_type_hashes() {
    use dora_ros2_bridge::{
        dora_message::descriptor::RmwZenohCompatibility,
        transport::{
            Ros2Qos, action::zenoh::resolve_action_endpoints,
            zenoh::compatibility::TypeDescriptionResolver,
        },
    };

    let resolver =
        TypeDescriptionResolver::from_prefixes([
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("msg-gen/test_type_descriptions")
        ]);
    let (keys, tokens) = resolve_action_endpoints(
        0,
        RmwZenohCompatibility::Rep2016,
        "/fibonacci",
        "example_interfaces",
        "Fibonacci",
        &Ros2Qos::default(),
        &resolver,
    )
    .unwrap();
    let send_goal = "RIHS01_d1a57fb2a4afe8c21e34fb10db206f16ce6729b28531141472df92277c55b557";
    let get_result = "RIHS01_1b0de0d5d29dc955d92f546706568428632771db13ec84c15ec1c1a59f424a57";
    let feedback = "RIHS01_c1de71afd52e49a89c53d8262366884185bc0a02f78ce051c4e46b0a7fe59bb2";
    let cancel_goal = "RIHS01_573d8b0a534451d7bc2ac8c5ffde8ac14b8593b7001175d0cd6516dcbeb8689a";
    let status = "RIHS01_6c1684b00f177d37438febe6e709fc4e2b0d4248dca4854946f9ed8b30cda83e";
    assert_eq!(tokens.send_goal.type_hash, send_goal);
    assert_eq!(tokens.get_result.type_hash, get_result);
    assert_eq!(tokens.feedback.type_hash, feedback);
    assert_eq!(tokens.cancel_goal.type_hash, cancel_goal);
    assert_eq!(tokens.status.type_hash, status);
    assert_eq!(
        keys.cancel_goal,
        format!(
            "0/fibonacci/_action/cancel_goal/action_msgs::srv::dds_::CancelGoal_/{cancel_goal}"
        )
    );
    assert_eq!(
        keys.status,
        format!("0/fibonacci/_action/status/action_msgs::msg::dds_::GoalStatusArray_/{status}")
    );
}

#[test]
fn concurrent_goal_limit_releases_completed_slots() {
    let limit = ConcurrentGoalLimit::new(2);
    let first = limit.try_acquire().unwrap();
    let second = limit.try_acquire().unwrap();
    assert!(matches!(
        limit.try_acquire(),
        Err(ActionError::GoalLimit { limit: 2 })
    ));
    drop(first);
    assert!(limit.try_acquire().is_ok());
    drop(second);
}

#[test]
fn shared_goal_slots_preserve_values_and_capacity() {
    let mut slots = GoalSlots::new(2);
    slots.insert("a", 10).unwrap();
    slots.insert("b", 20).unwrap();
    assert!(matches!(
        slots.insert("c", 30),
        Err(ActionError::GoalLimit { limit: 2 })
    ));
    assert_eq!(slots.get(&"a"), Some(&10));
    assert_eq!(slots.remove(&"a"), Some(10));
    assert_eq!(slots.len(), 1);
}

#[test]
fn accepted_rejected_and_concurrent_goals_obey_the_eight_goal_limit() {
    let mut state = ActionState::<Vec<u8>, Vec<u8>>::new(8);
    for id in 0u128..8 {
        state.accept(id.to_le_bytes(), vec![id as u8]).unwrap();
    }
    assert!(matches!(
        state.accept(8u128.to_le_bytes(), vec![8]),
        Err(ActionError::GoalLimit { limit: 8 })
    ));
    state.reject(9u128.to_le_bytes());
    assert_eq!(state.status(9u128.to_le_bytes()), None);
    assert_eq!(
        state.status(0u128.to_le_bytes()),
        Some(GoalStatus::Executing)
    );
}

#[test]
fn feedback_result_cancel_abort_and_unknown_goals_have_stable_transitions() {
    let mut state: ActionState<Vec<u8>, Vec<u8>> = ActionState::new(8);
    let goal = 1u128.to_le_bytes();
    state.accept(goal, vec![1]).unwrap();
    assert_eq!(state.feedback(goal, &[10]).unwrap(), 1);
    assert_eq!(state.feedback(goal, &[11]).unwrap(), 2);
    assert_eq!(
        state.request_result(goal).unwrap(),
        ResultAvailability::Pending
    );
    state.finish(goal, GoalStatus::Succeeded, vec![42]).unwrap();
    assert_eq!(
        state.request_result(goal).unwrap(),
        ResultAvailability::Ready {
            status: GoalStatus::Succeeded,
            result: vec![42]
        }
    );

    let canceled = 2u128.to_le_bytes();
    state.accept(canceled, vec![]).unwrap();
    state.cancel(canceled).unwrap();
    state
        .finish(canceled, GoalStatus::Canceled, vec![])
        .unwrap();
    assert_eq!(state.status(canceled), Some(GoalStatus::Canceled));

    let aborted = 3u128.to_le_bytes();
    state.accept(aborted, vec![]).unwrap();
    state.finish(aborted, GoalStatus::Aborted, vec![]).unwrap();
    assert!(matches!(
        state.feedback([99; 16], &[]),
        Err(ActionError::UnknownGoal)
    ));
}

#[test]
fn result_can_arrive_before_request_and_server_loss_releases_long_lived_waiters() {
    let mut state: ActionState<Vec<u8>, Vec<u8>> = ActionState::new(8);
    let completed = 4u128.to_le_bytes();
    state.accept(completed, vec![]).unwrap();
    state
        .finish(completed, GoalStatus::Succeeded, vec![7])
        .unwrap();
    assert!(
        matches!(state.request_result(completed).unwrap(), ResultAvailability::Ready { result, .. } if result == vec![7])
    );

    let waiting = 5u128.to_le_bytes();
    state.accept(waiting, vec![]).unwrap();
    assert_eq!(
        state.request_result(waiting).unwrap(),
        ResultAvailability::Pending
    );
    state.server_lost();
    assert_eq!(state.status(waiting), Some(GoalStatus::Aborted));
    assert!(matches!(
        state.request_result(waiting).unwrap(),
        ResultAvailability::ServerLost
    ));
}

/// `get_result` waits for a long-running goal, so it must not be capped by
/// Zenoh's `queries_default_timeout` (10 s by default) the way every action
/// call was before #3742.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn get_result_outlives_the_session_default_query_timeout() {
    use dora_ros2_bridge::transport::{
        Ros2Qos,
        action::{
            ActionEndpoint,
            zenoh::{ActionClient, ActionKeys, ActionServer, ActionTokens},
        },
        zenoh::{Context, ContextOptions, keyexpr::TopicToken},
    };
    use std::time::Duration;

    let config = std::env::temp_dir().join(format!(
        "dora-rmw-zenoh-short-query-timeout-{}.json5",
        std::process::id()
    ));
    std::fs::write(&config, "{ queries_default_timeout: 100 }").unwrap();
    let context = Context::open(ContextOptions {
        domain_id: 43,
        config_uri: Some(config.to_string_lossy().into_owned()),
    })
    .await;
    let _ = std::fs::remove_file(&config);
    let context = context.unwrap();
    let node = context
        .create_node("zid", "nid", "/", "/", "node")
        .await
        .unwrap();

    let endpoints = ActionEndpoints::new("/fibonacci", "example_interfaces", "Fibonacci");
    let key =
        |endpoint: &ActionEndpoint| format!("43{}/{}/hash", endpoint.name, endpoint.type_name);
    let token = |endpoint: &ActionEndpoint| TopicToken {
        name: endpoint.name.clone(),
        type_name: endpoint.type_name.clone(),
        type_hash: "hash".into(),
        qos: "2::,1:,:,:,,,".into(),
    };
    let keys = ActionKeys {
        send_goal: key(&endpoints.send_goal),
        get_result: key(&endpoints.get_result),
        cancel_goal: key(&endpoints.cancel_goal),
        feedback: key(&endpoints.feedback),
        status: key(&endpoints.status),
    };
    let tokens = ActionTokens {
        send_goal: token(&endpoints.send_goal),
        get_result: token(&endpoints.get_result),
        cancel_goal: token(&endpoints.cancel_goal),
        feedback: token(&endpoints.feedback),
        status: token(&endpoints.status),
    };
    let qos = Ros2Qos::default();
    let server = ActionServer::declare(&node, &keys, tokens.clone(), &qos)
        .await
        .unwrap();
    let client = ActionClient::declare(&node, &keys, tokens, &qos, 1024)
        .await
        .unwrap();

    let worker = tokio::spawn(async move {
        let request = server.get_result.recv().await.unwrap();
        tokio::time::sleep(Duration::from_millis(300)).await;
        server
            .get_result
            .reply(request.id, &request.payload)
            .await
            .unwrap();
    });
    let result = client
        .get_result
        .call(vec![0, 1, 0, 0, 5], Duration::from_secs(2))
        .await;
    assert_eq!(result.unwrap(), vec![0, 1, 0, 0, 5]);
    worker.await.unwrap();
}

/// Like `service_server_is_available` in rmw_zenoh, action readiness must not
/// require the server's endpoints to advertise the client's exact QoS.
#[tokio::test]
async fn wait_for_server_ignores_qos_like_rmw_zenoh() {
    use dora_ros2_bridge::transport::{
        action::zenoh::{ActionTokens, wait_for_server},
        zenoh::{
            graph::GraphCache,
            keyexpr::{EntityKind, LivelinessKey, TopicToken},
        },
    };
    use std::time::{Duration, Instant};

    let endpoints = ActionEndpoints::new("/fibonacci", "example_interfaces", "Fibonacci");
    let token =
        |endpoint: &dora_ros2_bridge::transport::action::ActionEndpoint, qos: &str| TopicToken {
            name: endpoint.name.clone(),
            type_name: endpoint.type_name.clone(),
            type_hash: "RIHS01_hash".into(),
            qos: qos.into(),
        };
    let tokens = |qos: &str| ActionTokens {
        send_goal: token(&endpoints.send_goal, qos),
        get_result: token(&endpoints.get_result, qos),
        cancel_goal: token(&endpoints.cancel_goal, qos),
        feedback: token(&endpoints.feedback, qos),
        status: token(&endpoints.status, qos),
    };

    // The server side, as an rclpy action server advertises it.
    let server = tokens("::,10:,:,:,,");
    let graph = GraphCache::new(7);
    for (index, (kind, topic)) in [
        (EntityKind::Service, &server.send_goal),
        (EntityKind::Service, &server.get_result),
        (EntityKind::Service, &server.cancel_goal),
        (EntityKind::Publisher, &server.feedback),
        (EntityKind::Publisher, &server.status),
    ]
    .into_iter()
    .enumerate()
    {
        let key = LivelinessKey::endpoint(
            7,
            "zid",
            "nid",
            &index.to_string(),
            kind,
            "/",
            "/",
            "node",
            topic.clone(),
        )
        .unwrap();
        graph.apply_put(key.as_str()).unwrap();
    }

    // A dora client asking with its own (different) QoS still finds it.
    wait_for_server(
        &graph,
        &tokens("2::,1:,:,:,,"),
        Instant::now() + Duration::from_millis(200),
    )
    .await
    .unwrap();

    // A different type identity is still a different action.
    let mut other_type = tokens("2::,1:,:,:,,");
    other_type.get_result.type_hash = "RIHS01_other".into();
    assert!(matches!(
        wait_for_server(
            &graph,
            &other_type,
            Instant::now() + Duration::from_millis(100)
        )
        .await,
        Err(dora_ros2_bridge::transport::action::zenoh::ActionTransportError::Timeout)
    ));
}

mod typed {
    //! The typed action client and server that the generated C++ bindings
    //! use on native Zenoh, run against each other in one process.

    use std::time::{Duration, Instant};

    use dora_ros2_bridge::{
        ros2_client::{action::GoalStatusEnum, action_msgs::CancelGoalResponseEnum},
        transport::{
            Ros2Qos,
            action::{
                ActionEndpoint, ActionEndpoints, ActionError,
                zenoh::{
                    ActionCallError, ActionKeys, ActionServerOptions, ActionTokens,
                    TypedActionClient, TypedActionServer,
                },
            },
            zenoh::{Context, ContextOptions, Node, keyexpr::TopicToken},
        },
    };
    use serde::{Deserialize, Serialize};

    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    struct Goal {
        order: i32,
    }

    #[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
    struct Sequence {
        sequence: Vec<i32>,
    }

    type Client = TypedActionClient<Goal, Sequence, Sequence>;
    type Server = TypedActionServer<Goal, Sequence, Sequence>;

    const TIMEOUT: Duration = Duration::from_secs(5);

    struct Fixture {
        // Keeps the Zenoh session open for the test. Bind it: fields a
        // `let Fixture { .. }` pattern leaves out are dropped right away.
        _session: Context,
        node: Node,
        client: Client,
        server: Server,
        pump: tokio::task::JoinHandle<()>,
    }

    async fn fixture(domain: usize, options: ActionServerOptions) -> Fixture {
        let context = Context::open(ContextOptions {
            domain_id: domain,
            config_uri: None,
        })
        .await
        .unwrap();
        let node = context
            .create_node("zid", "nid", "/", "/", "node")
            .await
            .unwrap();
        let endpoints = ActionEndpoints::new("/fibonacci", "example_interfaces", "Fibonacci");
        let key = |endpoint: &ActionEndpoint| {
            format!("{domain}{}/{}/hash", endpoint.name, endpoint.type_name)
        };
        let token = |endpoint: &ActionEndpoint| TopicToken {
            name: endpoint.name.clone(),
            type_name: endpoint.type_name.clone(),
            type_hash: "hash".into(),
            qos: "2::,1:,:,:,,,".into(),
        };
        let keys = ActionKeys {
            send_goal: key(&endpoints.send_goal),
            get_result: key(&endpoints.get_result),
            cancel_goal: key(&endpoints.cancel_goal),
            feedback: key(&endpoints.feedback),
            status: key(&endpoints.status),
        };
        let tokens = ActionTokens {
            send_goal: token(&endpoints.send_goal),
            get_result: token(&endpoints.get_result),
            cancel_goal: token(&endpoints.cancel_goal),
            feedback: token(&endpoints.feedback),
            status: token(&endpoints.status),
        };
        let qos = Ros2Qos::default();
        let (server, pump) = Server::declare(&node, &keys, tokens.clone(), &qos, options)
            .await
            .unwrap();
        let pump = tokio::spawn(pump);
        let client = Client::declare(&node, &keys, tokens, &qos, 1024 * 1024)
            .await
            .unwrap();
        client
            .wait_for_server(&node, Instant::now() + TIMEOUT)
            .await
            .unwrap();
        Fixture {
            _session: context,
            node,
            client,
            server,
            pump,
        }
    }

    fn options(max_goals: usize) -> ActionServerOptions {
        ActionServerOptions {
            max_goals,
            goal_timeout: Duration::from_secs(300),
        }
    }

    async fn next_goal(server: &Server) -> (dora_ros2_bridge::ros2_client::action::GoalId, Goal) {
        tokio::time::timeout(TIMEOUT, server.goals().recv_async())
            .await
            .expect("no goal event")
            .unwrap()
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn goal_feedback_status_and_result_round_trip() {
        let Fixture {
            client,
            server,
            _session,
            ..
        } = fixture(51, options(4)).await;

        let goal_id = client.send_goal(&Goal { order: 3 }, TIMEOUT).await.unwrap();
        let (received_id, goal) = next_goal(&server).await;
        assert_eq!(received_id, goal_id);
        assert_eq!(goal, Goal { order: 3 });
        assert_eq!(
            server.status(goal_id).await,
            Some(GoalStatusEnum::Executing)
        );

        server
            .publish_feedback(
                goal_id,
                &Sequence {
                    sequence: vec![0, 1],
                },
            )
            .await
            .unwrap();
        let (feedback_id, feedback) = tokio::time::timeout(TIMEOUT, client.recv_feedback())
            .await
            .unwrap()
            .unwrap();
        assert_eq!(feedback_id, goal_id);
        assert_eq!(feedback.sequence, vec![0, 1]);

        // The result is asked for before the goal finishes, as rclcpp does.
        // The pause makes that order all but certain; a request that came in
        // late would be answered from the stored result, which the next test
        // covers on its own.
        let result = tokio::spawn(async move {
            let result = client.get_result(goal_id).await;
            (client, result)
        });
        tokio::time::sleep(Duration::from_millis(200)).await;
        server
            .finish(
                goal_id,
                GoalStatusEnum::Succeeded,
                &Sequence {
                    sequence: vec![0, 1, 1],
                },
            )
            .await
            .unwrap();
        let (client, result) = tokio::time::timeout(TIMEOUT, result)
            .await
            .unwrap()
            .unwrap();
        let (status, result) = result.unwrap();
        assert!(matches!(status, GoalStatusEnum::Succeeded));
        assert_eq!(result.sequence, vec![0, 1, 1]);

        // Status is published after every change; the latest one shows the
        // goal as finished.
        let deadline = Instant::now() + TIMEOUT;
        loop {
            let list = tokio::time::timeout(TIMEOUT, client.recv_status())
                .await
                .unwrap()
                .unwrap();
            if list.iter().any(|status| {
                status.goal_info.goal_id == goal_id
                    && matches!(status.status, GoalStatusEnum::Succeeded)
            }) {
                break;
            }
            assert!(Instant::now() < deadline, "no Succeeded status");
        }
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn a_finished_result_can_be_fetched_later_and_more_than_once() {
        let Fixture {
            client,
            server,
            _session,
            ..
        } = fixture(52, options(4)).await;
        let goal_id = client.send_goal(&Goal { order: 1 }, TIMEOUT).await.unwrap();
        next_goal(&server).await;
        server
            .finish(
                goal_id,
                GoalStatusEnum::Aborted,
                &Sequence { sequence: vec![7] },
            )
            .await
            .unwrap();
        for _ in 0..2 {
            let (status, result) = client.get_result(goal_id).await.unwrap();
            assert!(matches!(status, GoalStatusEnum::Aborted));
            assert_eq!(result.sequence, vec![7]);
        }
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn finishing_twice_or_with_a_running_status_is_an_error() {
        let Fixture {
            client,
            server,
            _session,
            ..
        } = fixture(53, options(4)).await;
        let goal_id = client.send_goal(&Goal { order: 1 }, TIMEOUT).await.unwrap();
        next_goal(&server).await;
        let result = Sequence { sequence: vec![] };
        assert!(matches!(
            server
                .finish(goal_id, GoalStatusEnum::Executing, &result)
                .await,
            Err(ActionCallError::Goal(ActionError::InvalidTerminalStatus))
        ));
        server
            .finish(goal_id, GoalStatusEnum::Succeeded, &result)
            .await
            .unwrap();
        assert!(matches!(
            server
                .finish(goal_id, GoalStatusEnum::Succeeded, &result)
                .await,
            Err(ActionCallError::Goal(ActionError::TerminalGoal))
        ));
        assert!(matches!(
            server.publish_feedback(goal_id, &result).await,
            Err(ActionCallError::Goal(ActionError::TerminalGoal))
        ));
        let unknown = dora_ros2_bridge::ros2_client::action::GoalId::new_random();
        assert!(matches!(
            server
                .finish(unknown, GoalStatusEnum::Succeeded, &result)
                .await,
            Err(ActionCallError::Goal(ActionError::UnknownGoal))
        ));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn cancel_follows_the_ros_return_codes() {
        let Fixture {
            client,
            server,
            _session,
            ..
        } = fixture(54, options(4)).await;
        let goal_id = client.send_goal(&Goal { order: 1 }, TIMEOUT).await.unwrap();
        next_goal(&server).await;

        let response = client.cancel_goal(goal_id, TIMEOUT).await.unwrap();
        assert!(matches!(response.return_code, CancelGoalResponseEnum::None));
        assert_eq!(response.goals_canceling.len(), 1);
        assert_eq!(response.goals_canceling[0].goal_id, goal_id);
        assert_eq!(
            server.status(goal_id).await,
            Some(GoalStatusEnum::Canceling)
        );

        // Already canceling: like rcl_action, it can't be canceled again.
        let response = client.cancel_goal(goal_id, TIMEOUT).await.unwrap();
        assert!(matches!(
            response.return_code,
            CancelGoalResponseEnum::GoalTerminated
        ));

        server
            .finish(
                goal_id,
                GoalStatusEnum::Canceled,
                &Sequence { sequence: vec![] },
            )
            .await
            .unwrap();
        let response = client.cancel_goal(goal_id, TIMEOUT).await.unwrap();
        assert!(matches!(
            response.return_code,
            CancelGoalResponseEnum::GoalTerminated
        ));

        let unknown = dora_ros2_bridge::ros2_client::action::GoalId::new_random();
        let response = client.cancel_goal(unknown, TIMEOUT).await.unwrap();
        assert!(matches!(
            response.return_code,
            CancelGoalResponseEnum::UnknownGoal
        ));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn the_goal_limit_rejects_and_finished_goals_free_their_slot() {
        let Fixture {
            client,
            server,
            _session,
            ..
        } = fixture(55, options(1)).await;
        let first = client.send_goal(&Goal { order: 1 }, TIMEOUT).await.unwrap();
        next_goal(&server).await;
        assert!(matches!(
            client.send_goal(&Goal { order: 2 }, TIMEOUT).await,
            Err(ActionCallError::Rejected)
        ));

        // A finished goal nobody fetched yet gives up its slot to a new one.
        server
            .finish(
                first,
                GoalStatusEnum::Succeeded,
                &Sequence { sequence: vec![] },
            )
            .await
            .unwrap();
        let third = client.send_goal(&Goal { order: 3 }, TIMEOUT).await.unwrap();
        let (received, goal) = next_goal(&server).await;
        assert_eq!(received, third);
        assert_eq!(goal.order, 3);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn an_abandoned_goal_is_aborted_and_its_result_waiter_answered() {
        let Fixture {
            client,
            server,
            _session,
            ..
        } = fixture(
            56,
            ActionServerOptions {
                max_goals: 1,
                goal_timeout: Duration::from_millis(200),
            },
        )
        .await;
        let abandoned = client.send_goal(&Goal { order: 1 }, TIMEOUT).await.unwrap();
        next_goal(&server).await;

        // The server never finishes the goal. Without any further request it
        // is aborted after the timeout, and the client waiting for its result
        // gets `Aborted` with a default result, as from rclcpp.
        let (status, result) = tokio::time::timeout(TIMEOUT, client.get_result(abandoned))
            .await
            .expect("waiter never answered")
            .unwrap();
        assert!(matches!(status, GoalStatusEnum::Aborted));
        assert_eq!(result, Sequence::default());
        assert_eq!(
            server.status(abandoned).await,
            Some(GoalStatusEnum::Aborted)
        );
        assert!(matches!(
            server
                .finish(abandoned, GoalStatusEnum::Succeeded, &Sequence::default())
                .await,
            Err(ActionCallError::Goal(ActionError::TerminalGoal))
        ));

        // The aborted goal no longer blocks the only slot.
        let next = client.send_goal(&Goal { order: 2 }, TIMEOUT).await.unwrap();
        assert_eq!(next_goal(&server).await.0, next);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn an_unknown_goal_has_status_unknown_like_rclcpp() {
        let Fixture {
            client,
            server: _server,
            _session,
            ..
        } = fixture(58, options(4)).await;
        let unknown = dora_ros2_bridge::ros2_client::action::GoalId::new_random();
        let (status, result) = tokio::time::timeout(TIMEOUT, client.get_result(unknown))
            .await
            .unwrap()
            .unwrap();
        assert!(matches!(status, GoalStatusEnum::Unknown));
        assert_eq!(result, Sequence::default());
    }

    /// The generated C++ client calls `request_result` for every goal it
    /// sends; with as many goals in flight as the server takes, each waiting
    /// call must get its result.
    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn results_of_many_goals_can_be_awaited_at_once() {
        const GOALS: usize = 16;
        let Fixture {
            client,
            server,
            _session,
            ..
        } = fixture(59, options(GOALS)).await;
        let client = std::sync::Arc::new(client);
        let mut waiters = Vec::new();
        for order in 0..GOALS as i32 {
            let goal_id = client.send_goal(&Goal { order }, TIMEOUT).await.unwrap();
            let client = client.clone();
            waiters.push(tokio::spawn(async move {
                (order, client.get_result(goal_id).await)
            }));
        }
        tokio::time::sleep(Duration::from_millis(300)).await;
        for _ in 0..GOALS {
            let (goal_id, goal) = next_goal(&server).await;
            server
                .finish(
                    goal_id,
                    GoalStatusEnum::Succeeded,
                    &Sequence {
                        sequence: vec![goal.order],
                    },
                )
                .await
                .unwrap();
        }
        for waiter in waiters {
            let (order, result) = tokio::time::timeout(TIMEOUT, waiter)
                .await
                .unwrap()
                .unwrap();
            let (_, result) = result.unwrap_or_else(|e| panic!("goal {order}: {e}"));
            assert_eq!(result.sequence, vec![order]);
        }
    }

    /// Goals nobody reads stay queued, also after they are aborted or evicted,
    /// so the queue is what bounds them: with it full, a new goal is rejected.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn unread_goals_are_bounded_by_rejecting_new_ones() {
        let Fixture {
            client,
            server,
            _session,
            ..
        } = fixture(
            58,
            ActionServerOptions {
                max_goals: 1,
                goal_timeout: Duration::from_millis(50),
            },
        )
        .await;
        client.send_goal(&Goal { order: 1 }, TIMEOUT).await.unwrap();
        for order in 2..5 {
            // Long enough for the unread goal to be aborted, which makes it
            // evictable.
            tokio::time::sleep(Duration::from_millis(1200)).await;
            assert!(
                matches!(
                    client.send_goal(&Goal { order }, TIMEOUT).await,
                    Err(ActionCallError::Rejected)
                ),
                "goal {order} accepted while {} unread goals are queued",
                server.goals().len()
            );
        }
        assert_eq!(server.goals().len(), 1);
    }

    /// Closing the client ends a `get_result` still waiting for its goal, so
    /// the calls of an owner that is gone don't keep the client alive.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn closing_the_client_ends_a_waiting_get_result() {
        let Fixture {
            client,
            server,
            _session,
            ..
        } = fixture(60, options(4)).await;
        let client = std::sync::Arc::new(client);
        let goal = client.send_goal(&Goal { order: 1 }, TIMEOUT).await.unwrap();
        next_goal(&server).await;
        let waiting = tokio::spawn({
            let client = client.clone();
            async move { client.get_result(goal).await }
        });
        tokio::time::sleep(Duration::from_millis(200)).await;
        assert!(!waiting.is_finished(), "the goal is still running");
        client.close();
        let result = tokio::time::timeout(TIMEOUT, waiting)
            .await
            .expect("get_result kept waiting after close")
            .unwrap();
        assert!(matches!(result, Err(ActionCallError::Closed)), "{result:?}");

        // Later calls end at once without sending a request: if they did,
        // the server would keep each as a waiter, and once its 64 slots are
        // full the calls would fail with a timeout instead.
        for call in 0..200 {
            let result = client.get_result(goal).await;
            assert!(
                matches!(result, Err(ActionCallError::Closed)),
                "call {call} after close: {result:?}"
            );
        }
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn dropping_the_server_stops_it_and_leaves_the_graph() {
        let Fixture {
            client,
            server,
            pump,
            node,
            _session,
        } = fixture(57, options(4)).await;
        drop(server);
        tokio::time::timeout(TIMEOUT, pump)
            .await
            .expect("pump kept running")
            .unwrap();
        assert!(
            client
                .wait_for_server(&node, Instant::now() + Duration::from_millis(500))
                .await
                .is_err(),
            "server still in the ROS graph"
        );
    }
}
