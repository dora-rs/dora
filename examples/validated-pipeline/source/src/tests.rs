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

fn run_source(events: Vec<TimedIncomingEvent>) -> eyre::Result<Vec<serde_json::Value>> {
    let inputs = dora_node_api::integration_testing::TestingInput::Input(
        IntegrationTestInput::new("source".parse().unwrap(), events),
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

/// Sending 10 ticks must produce exactly 10 `value` outputs with the
/// monotonically increasing payload `0, 1, ..., 9`. This locks in both
/// the message count and the per-message content — the original smoke
/// test only checked that the dataflow exited without error.
#[test]
fn ten_ticks_produce_zero_through_nine() -> eyre::Result<()> {
    let events = (0..10)
        .map(|i| tick(0.001 * (i + 1) as f64))
        .collect::<Vec<_>>();

    let outputs = run_source(events)?;

    assert_eq!(outputs.len(), 10, "expected 10 outputs, got {outputs:?}");
    for (i, output) in outputs.iter().enumerate() {
        assert_eq!(output["id"], "value");
        assert_eq!(output["data_type"], "Int64");
        assert_eq!(output["data"], serde_json::json!([i as i64]));
    }
    Ok(())
}

/// Once `sent` has reached 10 the loop must exit even if more ticks are
/// pending. Sending 15 ticks must still produce exactly 10 outputs.
#[test]
fn loop_stops_after_ten_outputs() -> eyre::Result<()> {
    let events = (0..15)
        .map(|i| tick(0.001 * (i + 1) as f64))
        .collect::<Vec<_>>();

    let outputs = run_source(events)?;

    assert_eq!(outputs.len(), 10);
    Ok(())
}

/// A `Stop` event must terminate the loop early without producing more
/// outputs, even if `sent < 10`.
#[test]
fn stop_event_breaks_early() -> eyre::Result<()> {
    let events = vec![
        tick(0.001),
        tick(0.002),
        TimedIncomingEvent {
            time_offset_secs: 0.003,
            event: IncomingEvent::Stop,
        },
        tick(0.004),
    ];

    let outputs = run_source(events)?;

    assert_eq!(outputs.len(), 2);
    Ok(())
}

/// Non-`tick` inputs must be silently ignored — they must not consume
/// a slot in the `sent < 10` budget.
#[test]
fn unknown_inputs_are_ignored() -> eyre::Result<()> {
    let events = vec![
        TimedIncomingEvent {
            time_offset_secs: 0.001,
            event: IncomingEvent::Input {
                id: "noise".into(),
                metadata: None,
                data: None,
            },
        },
        tick(0.002),
        tick(0.003),
    ];

    let outputs = run_source(events)?;

    assert_eq!(outputs.len(), 2);
    assert_eq!(outputs[0]["data"], serde_json::json!([0_i64]));
    assert_eq!(outputs[1]["data"], serde_json::json!([1_i64]));
    Ok(())
}
