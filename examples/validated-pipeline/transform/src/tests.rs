use dora_node_api::{
    DoraNode, flume,
    integration_testing::{
        IntegrationTestInput,
        integration_testing_format::{IncomingEvent, InputData, TimedIncomingEvent},
    },
    serde_json,
};

fn value_input(time_offset_secs: f64, value: i64) -> TimedIncomingEvent {
    TimedIncomingEvent {
        time_offset_secs,
        event: IncomingEvent::Input {
            id: "value".into(),
            metadata: None,
            data: Some(Box::new(InputData::JsonObject {
                data: serde_json::json!([value]),
                data_type: Some(serde_json::json!("Int64")),
            })),
        },
    }
}

fn run_transform(events: Vec<TimedIncomingEvent>) -> eyre::Result<Vec<serde_json::Value>> {
    let inputs = dora_node_api::integration_testing::TestingInput::Input(
        IntegrationTestInput::new("transform".parse().unwrap(), events),
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

/// Each `value` input must produce exactly one `doubled` output whose
/// payload is the input multiplied by 2 — including the negative,
/// zero, and large-magnitude cases.
#[test]
fn doubles_each_value_input() -> eyre::Result<()> {
    let cases: &[i64] = &[0, 1, 7, -3, i64::MAX / 2];
    let events: Vec<_> = cases
        .iter()
        .enumerate()
        .map(|(i, v)| value_input(0.001 * (i + 1) as f64, *v))
        .collect();

    let outputs = run_transform(events)?;

    assert_eq!(outputs.len(), cases.len());
    for (output, expected) in outputs.iter().zip(cases.iter().map(|v| v * 2)) {
        assert_eq!(output["id"], "doubled");
        assert_eq!(output["data_type"], "Int64");
        assert_eq!(output["data"], serde_json::json!([expected]));
    }
    Ok(())
}

/// Inputs with a non-`value` id (and no `data`) must be ignored without
/// emitting a `doubled` output and without erroring out.
#[test]
fn ignores_non_value_inputs() -> eyre::Result<()> {
    let events = vec![
        TimedIncomingEvent {
            time_offset_secs: 0.001,
            event: IncomingEvent::Input {
                id: "tick".into(),
                metadata: None,
                data: None,
            },
        },
        value_input(0.002, 4),
        TimedIncomingEvent {
            time_offset_secs: 0.003,
            event: IncomingEvent::InputClosed { id: "tick".into() },
        },
    ];

    let outputs = run_transform(events)?;

    assert_eq!(outputs.len(), 1);
    assert_eq!(outputs[0]["data"], serde_json::json!([8_i64]));
    Ok(())
}

/// A non-`Int64` `value` payload must surface the documented
/// `"expected Int64Array"` context rather than panicking.
#[test]
fn rejects_non_int64_value_payload() {
    let events = vec![TimedIncomingEvent {
        time_offset_secs: 0.001,
        event: IncomingEvent::Input {
            id: "value".into(),
            metadata: None,
            data: Some(Box::new(InputData::JsonObject {
                data: serde_json::json!(["not a number"]),
                data_type: Some(serde_json::json!("Utf8")),
            })),
        },
    }];

    let err = run_transform(events).expect_err("non-Int64 value must produce an error");
    let chain: Vec<String> = err.chain().map(|e| e.to_string()).collect();
    assert!(
        chain.iter().any(|m| m.contains("expected Int64Array")),
        "error chain should mention `expected Int64Array`, got: {chain:?}"
    );
}
