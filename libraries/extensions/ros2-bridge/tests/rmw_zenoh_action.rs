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
