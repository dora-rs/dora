use dora_node_api::{
    DoraNode, flume,
    integration_testing::{
        IntegrationTestInput,
        integration_testing_format::{IncomingEvent, TimedIncomingEvent},
    },
    serde_json,
};

fn tick(time_offset_secs: f64) -> TimedIncomingEvent {
    TimedIncomingEvent {
        time_offset_secs,
        event: IncomingEvent::Input {
            id: "tick".into(),
            metadata: None,
            data: None,
        },
    }
}

fn run_sender(events: Vec<TimedIncomingEvent>) -> eyre::Result<Vec<serde_json::Value>> {
    let inputs = dora_node_api::integration_testing::TestingInput::Input(
        IntegrationTestInput::new("rust-sender".parse().unwrap(), events),
    );
    let (tx, rx) = flume::unbounded();
    let testing_output = dora_node_api::integration_testing::TestingOutput::ToChannel(tx);
    let (node, events) = DoraNode::init_testing(inputs, testing_output, Default::default())?;

    crate::run(node, events)?;

    Ok(rx
        .try_iter()
        .map(serde_json::Value::Object)
        .collect::<Vec<_>>())
}

/// Ten ticks must produce the exact sequence `0, 10, 20, ..., 90` as
/// `Int64` `values` outputs — this is the wire contract that the Python
/// receiver depends on. Locks both count and content.
#[test]
fn ten_ticks_produce_zero_through_ninety_step_ten() -> eyre::Result<()> {
    let events = (0..10)
        .map(|i| tick(0.001 * (i + 1) as f64))
        .collect::<Vec<_>>();

    let outputs = run_sender(events)?;

    assert_eq!(outputs.len(), 10);
    for (i, output) in outputs.iter().enumerate() {
        assert_eq!(output["id"], "values");
        assert_eq!(output["data_type"], "Int64");
        assert_eq!(output["data"], serde_json::json!([(i as i64) * 10]));
    }
    Ok(())
}

/// More than ten ticks must still produce exactly 10 outputs — the
/// `sent < 10` budget bounds the loop.
#[test]
fn loop_stops_after_ten_outputs() -> eyre::Result<()> {
    let events = (0..15)
        .map(|i| tick(0.001 * (i + 1) as f64))
        .collect::<Vec<_>>();

    let outputs = run_sender(events)?;

    assert_eq!(outputs.len(), 10);
    Ok(())
}
