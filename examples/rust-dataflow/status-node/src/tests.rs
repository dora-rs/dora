use std::{
    io::{Read, Seek},
    sync::Arc,
};

use dora_node_api::{
    DoraNode, flume,
    integration_testing::{
        self, IntegrationTestInput, TestingOptions,
        integration_testing_format::{IncomingEvent, InputData, TimedIncomingEvent},
    },
    serde_json,
};

#[test]
fn test_main_function() -> eyre::Result<()> {
    let inputs = dora_node_api::integration_testing::TestingInput::FromJsonFile(
        "../../../tests/sample-inputs/inputs-rust-status-node.json".into(),
    );
    let mut output_file = Arc::new(tempfile::tempfile()?);
    let testing_output =
        dora_node_api::integration_testing::TestingOutput::ToWriter(Box::new(output_file.clone()));
    let options = TestingOptions {
        skip_output_time_offsets: true,
    };

    integration_testing::setup_integration_testing(inputs, testing_output, options);

    crate::main()?;

    let mut output = String::new();
    output_file.seek(std::io::SeekFrom::Start(0))?;
    output_file.read_to_string(&mut output)?;
    let expected = std::fs::read_to_string(
        "../../../tests/sample-inputs/expected-outputs-rust-status-node.jsonl",
    )?;

    assert_eq!(output, expected.replace("\r\n", "\n")); // normalize line endings

    Ok(())
}

#[test]
fn test_run_function() -> eyre::Result<()> {
    let inputs = dora_node_api::integration_testing::TestingInput::FromJsonFile(
        "../../../tests/sample-inputs/inputs-rust-status-node.json".into(),
    );
    let mut output_file = Arc::new(tempfile::tempfile()?);
    let testing_output =
        dora_node_api::integration_testing::TestingOutput::ToWriter(Box::new(output_file.clone()));
    let options = TestingOptions {
        skip_output_time_offsets: true,
    };

    let (node, events) = DoraNode::init_testing(inputs, testing_output, options)?;

    crate::run(node, events)?;

    let mut output = String::new();
    output_file.seek(std::io::SeekFrom::Start(0))?;
    output_file.read_to_string(&mut output)?;
    let expected = std::fs::read_to_string(
        "../../../tests/sample-inputs/expected-outputs-rust-status-node.jsonl",
    )?;

    assert_eq!(output, expected.replace("\r\n", "\n")); // normalize line endings

    Ok(())
}

fn random_input(time_offset_secs: f64, value: u64) -> TimedIncomingEvent {
    TimedIncomingEvent {
        time_offset_secs,
        event: IncomingEvent::Input {
            id: "random".into(),
            metadata: None,
            data: Some(Box::new(InputData::JsonObject {
                data: serde_json::json!([value]),
                data_type: Some(serde_json::json!("UInt64")),
            })),
        },
    }
}

fn tick_input(time_offset_secs: f64) -> TimedIncomingEvent {
    TimedIncomingEvent {
        time_offset_secs,
        event: IncomingEvent::Input {
            id: "tick".into(),
            metadata: None,
            data: None,
        },
    }
}

/// Drives `run` with a hand-built event sequence and asserts the exact
/// `status` strings — including that the embedded `after N ticks` count
/// reflects the number of `tick` events that arrived *before* each
/// `random` event, and that `InputClosed { id: "random" }` causes the
/// loop to exit cleanly.
#[test]
fn test_tick_counter_and_random_round_trip() -> eyre::Result<()> {
    let events = vec![
        random_input(0.001, 0xAA),   // 0 ticks so far
        tick_input(0.010),           // ticks -> 1
        random_input(0.011, 0xBB),   // 1 tick
        tick_input(0.020),           // ticks -> 2
        tick_input(0.021),           // ticks -> 3
        random_input(0.022, 0xCCDD), // 3 ticks
        TimedIncomingEvent {
            time_offset_secs: 0.030,
            event: IncomingEvent::InputClosed {
                id: "random".into(),
            },
        },
    ];
    let inputs = dora_node_api::integration_testing::TestingInput::Input(
        IntegrationTestInput::new("rust-status-node".parse().unwrap(), events),
    );

    let (tx, rx) = flume::unbounded();
    let testing_output = dora_node_api::integration_testing::TestingOutput::ToChannel(tx);
    let (node, events) = DoraNode::init_testing(inputs, testing_output, Default::default())?;

    crate::run(node, events)?;

    let outputs = rx.try_iter().collect::<Vec<_>>();
    let expected_messages = [
        "operator received random value 0xaa after 0 ticks",
        "operator received random value 0xbb after 1 ticks",
        "operator received random value 0xccdd after 3 ticks",
    ];

    assert_eq!(
        outputs.len(),
        expected_messages.len(),
        "unexpected number of `status` outputs: {outputs:?}"
    );
    for (output, expected) in outputs.iter().zip(expected_messages.iter()) {
        assert_eq!(output["id"], "status");
        assert_eq!(output["data_type"], "Utf8");
        assert_eq!(output["data"], serde_json::json!([expected]));
    }

    Ok(())
}

/// Sending a `random` input with a non-`UInt64` payload must surface the
/// documented `"unexpected data type"` error rather than silently coercing
/// or panicking. Locks in the `u64::try_from(&data).context(...)` contract.
#[test]
fn test_non_uint64_random_input_errors() -> eyre::Result<()> {
    let events = vec![TimedIncomingEvent {
        time_offset_secs: 0.001,
        event: IncomingEvent::Input {
            id: "random".into(),
            metadata: None,
            data: Some(Box::new(InputData::JsonObject {
                data: serde_json::json!(["not a number"]),
                data_type: Some(serde_json::json!("Utf8")),
            })),
        },
    }];
    let inputs = dora_node_api::integration_testing::TestingInput::Input(
        IntegrationTestInput::new("rust-status-node".parse().unwrap(), events),
    );

    let (tx, _rx) = flume::unbounded();
    let testing_output = dora_node_api::integration_testing::TestingOutput::ToChannel(tx);
    let (node, events) = DoraNode::init_testing(inputs, testing_output, Default::default())?;

    let err = crate::run(node, events).expect_err("non-UInt64 random input must produce an error");
    let chain: Vec<String> = err.chain().map(|e| e.to_string()).collect();
    assert!(
        chain.iter().any(|m| m.contains("unexpected data type")),
        "error chain should mention the type-mismatch context, got: {chain:?}"
    );

    Ok(())
}
