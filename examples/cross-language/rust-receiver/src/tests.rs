use dora_node_api::{
    DoraNode, flume,
    integration_testing::{
        IntegrationTestInput,
        integration_testing_format::{IncomingEvent, InputData, TimedIncomingEvent},
    },
    serde_json,
};

fn values_input(time_offset_secs: f64, value: i64) -> TimedIncomingEvent {
    TimedIncomingEvent {
        time_offset_secs,
        event: IncomingEvent::Input {
            id: "values".into(),
            metadata: None,
            data: Some(Box::new(InputData::JsonObject {
                data: serde_json::json!([value]),
                data_type: Some(serde_json::json!("Int64")),
            })),
        },
    }
}

fn run_receiver(events: Vec<TimedIncomingEvent>) -> eyre::Result<()> {
    let inputs = dora_node_api::integration_testing::TestingInput::Input(
        IntegrationTestInput::new("rust-receiver".parse().unwrap(), events),
    );
    let (tx, _rx) = flume::unbounded();
    let testing_output = dora_node_api::integration_testing::TestingOutput::ToChannel(tx);
    let (node, events) = DoraNode::init_testing(inputs, testing_output, Default::default())?;

    crate::run(node, events)
}

/// The full simulated Python output (`0, 10, ..., 90`) must be accepted —
/// every value satisfies the modulo + range contract and the count is >= 5.
#[test]
fn accepts_full_python_sequence() -> eyre::Result<()> {
    let events = (0..10)
        .map(|i| values_input(0.001 * (i + 1) as f64, i * 10))
        .collect::<Vec<_>>();

    run_receiver(events)
}

/// A value outside the `[0, 90]` range must trip the documented modulo/range
/// guard with the exact error message — not be silently accepted.
#[test]
fn rejects_value_out_of_range() {
    let events = vec![
        values_input(0.001, 0),
        values_input(0.002, 100), // outside [0, 90]
    ];

    let err = run_receiver(events).expect_err("out-of-range value must error");
    let chain: Vec<String> = err.chain().map(|e| e.to_string()).collect();
    assert!(
        chain
            .iter()
            .any(|m| m.contains("unexpected value from Python sender")),
        "error chain should call out the bad value, got: {chain:?}"
    );
}

/// A value that is in range but not a multiple of 10 must also be rejected.
#[test]
fn rejects_non_multiple_of_ten() {
    let events = vec![values_input(0.001, 7)];

    let err = run_receiver(events).expect_err("non-multiple-of-10 value must error");
    let chain: Vec<String> = err.chain().map(|e| e.to_string()).collect();
    assert!(
        chain
            .iter()
            .any(|m| m.contains("unexpected value from Python sender")),
        "error chain should call out the bad value, got: {chain:?}"
    );
}

/// Fewer than 5 messages followed by `Stop` must trigger the final
/// `received_count < 5` guard.
#[test]
fn rejects_too_few_messages() {
    let events = vec![
        values_input(0.001, 0),
        values_input(0.002, 10),
        TimedIncomingEvent {
            time_offset_secs: 0.003,
            event: IncomingEvent::Stop,
        },
    ];

    let err = run_receiver(events).expect_err("must require at least 5 messages");
    let chain: Vec<String> = err.chain().map(|e| e.to_string()).collect();
    assert!(
        chain
            .iter()
            .any(|m| m.contains("got only 2") && m.contains(">= 5")),
        "error chain should mention the minimum-message contract, got: {chain:?}"
    );
}

/// A non-`Int64` payload must surface `"expected Int64Array from Python sender"`
/// rather than panicking on a downcast failure.
#[test]
fn rejects_non_int64_payload() {
    let events = vec![TimedIncomingEvent {
        time_offset_secs: 0.001,
        event: IncomingEvent::Input {
            id: "values".into(),
            metadata: None,
            data: Some(Box::new(InputData::JsonObject {
                data: serde_json::json!(["not a number"]),
                data_type: Some(serde_json::json!("Utf8")),
            })),
        },
    }];

    let err = run_receiver(events).expect_err("non-Int64 input must error");
    let chain: Vec<String> = err.chain().map(|e| e.to_string()).collect();
    assert!(
        chain
            .iter()
            .any(|m| m.contains("expected Int64Array from Python sender")),
        "error chain should mention the Int64 type contract, got: {chain:?}"
    );
}

/// A multi-element array (Python sends single-element arrays) must trip the
/// `expected 1 element` guard. Locks in the contract that this rust-receiver
/// is intentionally strict about element count.
#[test]
fn rejects_multi_element_array() {
    let events = vec![TimedIncomingEvent {
        time_offset_secs: 0.001,
        event: IncomingEvent::Input {
            id: "values".into(),
            metadata: None,
            data: Some(Box::new(InputData::JsonObject {
                data: serde_json::json!([0, 10]),
                data_type: Some(serde_json::json!("Int64")),
            })),
        },
    }];

    let err = run_receiver(events).expect_err("multi-element array must error");
    let chain: Vec<String> = err.chain().map(|e| e.to_string()).collect();
    assert!(
        chain
            .iter()
            .any(|m| m.contains("expected 1 element from Python")),
        "error chain should mention the element-count contract, got: {chain:?}"
    );
}
