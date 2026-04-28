use dora_node_api::{
    DoraNode, flume,
    integration_testing::{
        IntegrationTestInput,
        integration_testing_format::{IncomingEvent, InputData, TimedIncomingEvent},
    },
    serde_json,
};

fn message_input(time_offset_secs: f64, message: &str) -> TimedIncomingEvent {
    TimedIncomingEvent {
        time_offset_secs,
        event: IncomingEvent::Input {
            id: "message".into(),
            metadata: None,
            data: Some(Box::new(InputData::JsonObject {
                data: serde_json::json!([message]),
                data_type: Some(serde_json::json!("Utf8")),
            })),
        },
    }
}

fn run_sink(events: Vec<TimedIncomingEvent>) -> eyre::Result<()> {
    let inputs = dora_node_api::integration_testing::TestingInput::Input(
        IntegrationTestInput::new("multiple-daemons-sink".parse().unwrap(), events),
    );
    let (tx, _rx) = flume::unbounded();
    let testing_output = dora_node_api::integration_testing::TestingOutput::ToChannel(tx);
    let (node, events) = DoraNode::init_testing(inputs, testing_output, Default::default())?;

    crate::run(node, events)
}

/// A stream of well-formed status messages must run to completion without
/// error. Locks in the contract that any string matching
/// `"operator received random value … ticks"` is accepted.
#[test]
fn accepts_valid_status_messages() -> eyre::Result<()> {
    let events = vec![
        message_input(
            0.001,
            "operator received random value 0xdeadbeef after 0 ticks",
        ),
        message_input(0.002, "operator received random value 0xcafe after 7 ticks"),
        TimedIncomingEvent {
            time_offset_secs: 0.003,
            event: IncomingEvent::InputClosed {
                id: "message".into(),
            },
        },
    ];

    run_sink(events)
}

/// A message missing the required prefix must produce an error mentioning
/// the prefix contract — not a generic `Ok(())` or empty error string.
#[test]
fn rejects_message_missing_prefix() {
    let events = vec![message_input(0.001, "totally unrelated payload")];

    let err = run_sink(events).expect_err("sink must reject malformed messages");
    let chain: Vec<String> = err.chain().map(|e| e.to_string()).collect();
    assert!(
        chain
            .iter()
            .any(|m| m.contains("operator received random value")),
        "error chain should mention the required prefix, got: {chain:?}"
    );
}

/// A message that has the right prefix but the wrong suffix must trip the
/// second `bail!` ("should end with 'ticks'"), so we cover both branches.
#[test]
fn rejects_message_with_wrong_suffix() {
    let events = vec![message_input(
        0.001,
        "operator received random value 0xff after seven somethings",
    )];

    let err = run_sink(events).expect_err("sink must reject messages without a `ticks` suffix");
    let chain: Vec<String> = err.chain().map(|e| e.to_string()).collect();
    assert!(
        chain.iter().any(|m| m.contains("ticks")),
        "error chain should mention the `ticks` suffix contract, got: {chain:?}"
    );
}

/// A non-string `message` payload must surface the `"expected string message"`
/// context rather than panicking on an Arrow type mismatch.
#[test]
fn rejects_non_string_message_payload() {
    let events = vec![TimedIncomingEvent {
        time_offset_secs: 0.001,
        event: IncomingEvent::Input {
            id: "message".into(),
            metadata: None,
            data: Some(Box::new(InputData::JsonObject {
                data: serde_json::json!([42_u64]),
                data_type: Some(serde_json::json!("UInt64")),
            })),
        },
    }];

    let err = run_sink(events).expect_err("sink must reject non-string `message` payloads");
    let chain: Vec<String> = err.chain().map(|e| e.to_string()).collect();
    assert!(
        chain.iter().any(|m| m.contains("expected string message")),
        "error chain should mention the type-mismatch context, got: {chain:?}"
    );
}

/// Non-`message` inputs are logged and skipped — they must not produce
/// an error and must not stop the loop.
#[test]
fn non_message_inputs_are_ignored() -> eyre::Result<()> {
    let events = vec![
        TimedIncomingEvent {
            time_offset_secs: 0.001,
            event: IncomingEvent::Input {
                id: "noise".into(),
                metadata: None,
                data: None,
            },
        },
        message_input(0.002, "operator received random value 0x1 after 0 ticks"),
    ];

    run_sink(events)
}
