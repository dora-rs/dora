use std::{
    io::{Read, Seek},
    sync::Arc,
};

use dora_node_api::{
    DoraNode, Event, IntoArrow,
    arrow::array::{Array, NullArray},
    dora_core::config::DataId,
    flume,
    integration_testing::{
        IntegrationTestInput, TestingOptions,
        integration_testing_format::{IncomingEvent, TimedIncomingEvent},
    },
    serde_json,
};

#[test]
fn test_run_function() -> eyre::Result<()> {
    let inputs = dora_node_api::integration_testing::TestingInput::FromJsonFile(
        "../../../tests/sample-inputs/inputs-rust-node.json".into(),
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
    let expected =
        std::fs::read_to_string("../../../tests/sample-inputs/expected-outputs-rust-node.jsonl")?;

    assert_eq!(output, expected);

    Ok(())
}

#[test]
fn test_sample_output() -> eyre::Result<()> {
    let events = vec![
        TimedIncomingEvent {
            time_offset_secs: 0.01,
            event: IncomingEvent::Input {
                id: "tick".into(),
                metadata: None,
                data: None,
            },
        },
        TimedIncomingEvent {
            time_offset_secs: 0.05,
            event: IncomingEvent::Input {
                id: "tick".into(),
                metadata: None,
                data: None,
            },
        },
        TimedIncomingEvent {
            time_offset_secs: 0.055,
            event: IncomingEvent::Stop,
        },
    ];
    let inputs = dora_node_api::integration_testing::TestingInput::Input(
        IntegrationTestInput::new("node_id".parse().unwrap(), events),
    );

    let (tx, rx) = flume::unbounded();
    let testing_output = dora_node_api::integration_testing::TestingOutput::ToChannel(tx);
    let (mut node, mut events) =
        DoraNode::init_testing(inputs, testing_output, Default::default())?;

    let output = DataId::from("i");

    for i in 0.. {
        let event = match events.recv() {
            Some(input) => input,
            None => break,
        };

        match event {
            Event::Input { id, metadata, data } => match id.as_str() {
                "tick" => {
                    assert_eq!(data.to_data(), NullArray::new(0).to_data());
                    node.send_output(output.clone(), metadata.parameters, i.into_arrow())?;
                }
                other => panic!("unexpected input `{other}`"),
            },
            Event::Stop(_) => {
                node.send_output(DataId::from("stop"), Default::default(), i.into_arrow())?;
            }
            _ => {}
        }
    }

    std::mem::drop(node);
    std::mem::drop(events);

    let outputs = rx.try_iter().collect::<Vec<_>>();
    let expected: Vec<serde_json::Map<_, _>> = [
        serde_json::json!({
            "id": "i",
            "min_time_offset_secs": 0.01,
            "data_type": "Int32",
            "data": [0]
        }),
        serde_json::json!({
            "id": "i",
            "min_time_offset_secs": 0.05,
            "data_type": "Int32",
            "data": [1]
        }),
        serde_json::json!({
            "id": "stop",
            "min_time_offset_secs": 0.055,
            "data_type": "Int32",
            "data": [2]
        }),
    ]
    .into_iter()
    .map(|v| v.as_object().unwrap().clone())
    .collect();

    assert_eq!(outputs.len(), expected.len());
    let mut time_offset_buf = 0.0;
    for (output, expected) in outputs.iter().zip(expected.iter()) {
        assert_eq!(output["id"], expected["id"]);
        assert_eq!(output["data"], expected["data"]);
        assert_eq!(output["data_type"], expected["data_type"]);

        assert!(output["time_offset_secs"].as_f64().unwrap() >= time_offset_buf);
        assert!(
            output["time_offset_secs"].as_f64().unwrap()
                >= expected["min_time_offset_secs"].as_f64().unwrap()
        );

        time_offset_buf = output["time_offset_secs"].as_f64().unwrap();
    }

    Ok(())
}
