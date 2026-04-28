use dora_node_api::{
    DoraNode, flume,
    integration_testing::{
        IntegrationTestInput,
        integration_testing_format::{IncomingEvent, InputData, TimedIncomingEvent},
    },
    serde_json,
};

fn doubled_input(time_offset_secs: f64, value: i64) -> TimedIncomingEvent {
    TimedIncomingEvent {
        time_offset_secs,
        event: IncomingEvent::Input {
            id: "doubled".into(),
            metadata: None,
            data: Some(Box::new(InputData::JsonObject {
                data: serde_json::json!([value]),
                data_type: Some(serde_json::json!("Int64")),
            })),
        },
    }
}

fn run_sink(events: Vec<TimedIncomingEvent>) -> eyre::Result<()> {
    let inputs = dora_node_api::integration_testing::TestingInput::Input(
        IntegrationTestInput::new("sink".parse().unwrap(), events),
    );
    let (tx, _rx) = flume::unbounded();
    let testing_output = dora_node_api::integration_testing::TestingOutput::ToChannel(tx);
    let (node, events) = DoraNode::init_testing(inputs, testing_output, Default::default())?;

    crate::run(node, events)
}

/// Five sequential doubled values (`0, 2, 4, 6, 8`) match the `received * 2`
/// invariant and meet the `received >= 5` minimum, so the run must succeed.
#[test]
fn accepts_valid_doubled_sequence() -> eyre::Result<()> {
    let events = (0..5)
        .map(|i| doubled_input(0.001 * (i + 1) as f64, (i as i64) * 2))
        .collect::<Vec<_>>();

    run_sink(events)
}

/// Skipping a value (sending `4` instead of `2` for the second message)
/// must trip the `expected/got` invariant rather than passing silently.
#[test]
fn rejects_out_of_order_doubled_value() {
    let events = vec![doubled_input(0.001, 0), doubled_input(0.002, 4)];

    let err = run_sink(events).expect_err("sink must reject out-of-order values");
    let chain: Vec<String> = err.chain().map(|e| e.to_string()).collect();
    assert!(
        chain.iter().any(|m| m.contains("expected 2, got 4")),
        "error chain should mention the invariant violation, got: {chain:?}"
    );
}

/// Receiving fewer than 5 messages and then closing the stream must fail
/// the final `received < 5` guard, even if every individual value matched.
#[test]
fn rejects_too_few_messages() {
    let events = vec![
        doubled_input(0.001, 0),
        doubled_input(0.002, 2),
        doubled_input(0.003, 4),
    ];

    let err = run_sink(events).expect_err("sink must require >= 5 messages");
    let chain: Vec<String> = err.chain().map(|e| e.to_string()).collect();
    assert!(
        chain
            .iter()
            .any(|m| m.contains("got only 3") && m.contains(">= 5")),
        "error chain should mention the minimum-message contract, got: {chain:?}"
    );
}

/// A non-`Int64` `doubled` payload must surface the documented
/// `"expected Int64Array"` context rather than panicking.
#[test]
fn rejects_non_int64_doubled_payload() {
    let events = vec![TimedIncomingEvent {
        time_offset_secs: 0.001,
        event: IncomingEvent::Input {
            id: "doubled".into(),
            metadata: None,
            data: Some(Box::new(InputData::JsonObject {
                data: serde_json::json!(["nope"]),
                data_type: Some(serde_json::json!("Utf8")),
            })),
        },
    }];

    let err = run_sink(events).expect_err("non-Int64 input must produce an error");
    let chain: Vec<String> = err.chain().map(|e| e.to_string()).collect();
    assert!(
        chain.iter().any(|m| m.contains("expected Int64Array")),
        "error chain should mention the type-mismatch context, got: {chain:?}"
    );
}

/// A `Stop` event must terminate the loop, after which the `received >= 5`
/// guard still runs — so a happy 5-message stream followed by `Stop` is
/// accepted, but the trailing `Stop` doesn't bypass the count check.
#[test]
fn stop_event_terminates_loop_then_runs_min_check() -> eyre::Result<()> {
    let mut events: Vec<_> = (0..5)
        .map(|i| doubled_input(0.001 * (i + 1) as f64, (i as i64) * 2))
        .collect();
    events.push(TimedIncomingEvent {
        time_offset_secs: 0.010,
        event: IncomingEvent::Stop,
    });

    run_sink(events)
}
