//! Checks on the generated C++ action bindings.

use dora_ros2_bridge_msg_gen::{
    format_token_stream,
    types::{Action, Message},
};
use quote::ToTokens as _;

fn empty_msg(name: &str) -> Message {
    Message {
        package: "test_pkg".to_string(),
        name: name.to_string(),
        members: vec![],
        constants: vec![],
    }
}

/// A `request_result` sent before the result service is matched is lost on
/// DDS: the client never gets a result and the server waits in `send_result`.
/// So `wait_for_action` has to wait for every part of the action server, like
/// `rcl_action_server_is_available`, not only for the goal service.
#[test]
fn wait_for_action_waits_for_every_action_endpoint() {
    let action = Action {
        package: "test_pkg".to_string(),
        name: "Fibonacci".to_string(),
        goal: empty_msg("Fibonacci_Goal"),
        result: empty_msg("Fibonacci_Result"),
        feedback: empty_msg("Fibonacci_Feedback"),
    };
    let (_decl, imp) = action.cxx_action_creation_functions("test_pkg");
    let client = format_token_stream(imp.into_token_stream());
    let start = client
        .find("fn wait_for__test_pkg__Fibonacci")
        .expect("generated wait_for_action");
    let wait = &client[start..];
    let wait = &wait[..wait[1..]
        .find("\n    fn ")
        .map_or(wait.len(), |end| end + 1)];
    for endpoint in [
        "goal_client().wait_for_service",
        "result_client().wait_for_service",
        "cancel_client().wait_for_service",
        "feedback_subscription().wait_for_publisher",
        "status_subscription().wait_for_publisher",
    ] {
        assert!(
            wait.contains(endpoint),
            "wait_for_action must wait for {endpoint}"
        );
    }
}
