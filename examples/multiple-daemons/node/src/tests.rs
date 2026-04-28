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

fn run_node(events: Vec<TimedIncomingEvent>) -> eyre::Result<Vec<serde_json::Value>> {
    let inputs = dora_node_api::integration_testing::TestingInput::Input(
        IntegrationTestInput::new("multiple-daemons-node".parse().unwrap(), events),
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

/// Each `tick` input must produce exactly one `random` output of type
/// `UInt64`. We can't pin the value (it's `rand::random()`) but we can
/// pin the count, the output id, and the data type — none of which the
/// liveness-only smoke test caught.
#[test]
fn each_tick_produces_one_uint64_random_output() -> eyre::Result<()> {
    let events = (0..7)
        .map(|i| tick(0.001 * (i + 1) as f64))
        .collect::<Vec<_>>();

    let outputs = run_node(events)?;

    assert_eq!(outputs.len(), 7, "expected 7 random outputs");
    for output in &outputs {
        assert_eq!(output["id"], "random");
        assert_eq!(output["data_type"], "UInt64");
        // payload must be a single-element array (rand::random::<u64>())
        let arr = output["data"]
            .as_array()
            .expect("data must be a JSON array");
        assert_eq!(arr.len(), 1, "each output must wrap a single u64");
    }
    Ok(())
}

/// Non-`tick` inputs are logged and skipped — they must not produce a
/// `random` output and must not error.
#[test]
fn non_tick_inputs_are_ignored() -> eyre::Result<()> {
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
    ];

    let outputs = run_node(events)?;

    assert_eq!(outputs.len(), 1);
    Ok(())
}

/// `Stop` ends the event stream — anything queued after `Stop` must
/// not reach the loop. Locks in the upstream contract that `Event::Stop`
/// terminates the inner `EventStream`, so the trailing tick is dropped.
#[test]
fn stop_event_terminates_stream() -> eyre::Result<()> {
    let events = vec![
        tick(0.001),
        TimedIncomingEvent {
            time_offset_secs: 0.002,
            event: IncomingEvent::Stop,
        },
        tick(0.003),
    ];

    let outputs = run_node(events)?;

    assert_eq!(
        outputs.len(),
        1,
        "tick after Stop should be dropped: {outputs:?}"
    );
    Ok(())
}

/// `InputClosed` (without a `Stop`) must not end the loop — subsequent
/// ticks should still produce outputs.
#[test]
fn input_closed_does_not_terminate_loop() -> eyre::Result<()> {
    let events = vec![
        tick(0.001),
        TimedIncomingEvent {
            time_offset_secs: 0.002,
            event: IncomingEvent::InputClosed { id: "tick".into() },
        },
        tick(0.003),
    ];

    let outputs = run_node(events)?;

    assert_eq!(outputs.len(), 2, "InputClosed must not stop the loop");
    Ok(())
}

/// The hard cap of 100 iterations must bound the total number of `random`
/// outputs even when more than 100 ticks are queued.
#[test]
fn loop_stops_after_one_hundred_iterations() -> eyre::Result<()> {
    let events = (0..150)
        .map(|i| tick(0.0001 * (i + 1) as f64))
        .collect::<Vec<_>>();

    let outputs = run_node(events)?;

    assert_eq!(
        outputs.len(),
        100,
        "loop should terminate after exactly 100 iterations"
    );
    // sanity: ensure at least two outputs differ — random shouldn't be constant
    let first = &outputs[0]["data"];
    let differs = outputs.iter().skip(1).any(|o| &o["data"] != first);
    assert!(
        differs,
        "rand::random outputs should not all be equal: {outputs:?}"
    );
    Ok(())
}
