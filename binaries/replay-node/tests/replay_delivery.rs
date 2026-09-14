use std::{
    collections::BTreeMap,
    fs::{self, File},
    path::{Path, PathBuf},
    process::{Command, ExitStatus, Stdio},
    thread,
    time::{Duration, Instant},
};

use aligned_vec::{AVec, ConstAlign};
use dora_message::{
    DataflowId,
    common::Timestamped,
    daemon_to_daemon::InterDaemonEvent,
    id::{DataId, NodeId},
    metadata::{Metadata, Parameter},
    uhlc::HLC,
};
use dora_node_api::{
    DoraNode, Event, IntoArrow,
    arrow_utils::{decode_arrow_ipc, encode_arrow_ipc},
    arrow_v59::array::Int64Array,
};
use dora_recording::{
    FORMAT_VERSION, RecordEntry, RecordingHeader, RecordingReader, RecordingWriter,
};
use eyre::{Context, ContextCompat};

#[cfg(unix)]
use std::os::unix::process::CommandExt;

const MESSAGE_COUNT: usize = 100;
const SOURCE_ID: &str = "recorded-source";
const OUTPUT_ID: &str = "samples";
const SATURATION_COUNT: usize = 7;
const SATURATION_PAYLOAD_ELEMENTS: usize = 5 * 1024 * 1024;
const MULTI_OUTPUT_COUNT: usize = 6;

#[derive(Clone, Copy, PartialEq)]
enum SinkBehavior {
    Drain,
    Stop,
    MissingSuffix,
    MissingReceipt,
    Stall,
    LargePayload,
    LargeMetadata,
    ByteSaturation,
    #[cfg(unix)]
    Cancel,
}

impl SinkBehavior {
    fn name(self) -> &'static str {
        match self {
            Self::Drain => "drain",
            Self::Stop => "stop",
            Self::MissingSuffix => "missing_suffix",
            Self::MissingReceipt => "missing_receipt",
            Self::Stall => "stall",
            Self::LargePayload => "large_payload",
            Self::LargeMetadata => "large_metadata",
            Self::ByteSaturation => "byte_saturation",
            #[cfg(unix)]
            Self::Cancel => "cancel",
        }
    }

    fn count(self) -> usize {
        match self {
            Self::LargePayload | Self::LargeMetadata => 8,
            Self::ByteSaturation => SATURATION_COUNT,
            _ => MESSAGE_COUNT,
        }
    }

    fn payload_elements(self) -> usize {
        match self {
            Self::LargePayload => 2 * 1024 * 1024,
            Self::ByteSaturation => SATURATION_PAYLOAD_ELEMENTS,
            _ => 2,
        }
    }

    fn metadata_bytes(self) -> usize {
        if self == Self::LargeMetadata {
            16 * 1024 * 1024
        } else {
            0
        }
    }

    fn expects_success(self) -> bool {
        matches!(self, Self::Drain | Self::LargePayload | Self::LargeMetadata)
    }
}

#[test]
#[ignore = "requires DORA_REPLAY_TEST_CLI built from this checkout"]
fn speed_zero_delivers_ordered_recording() -> eyre::Result<()> {
    run_test_case(SinkBehavior::Drain)
}

#[test]
#[ignore = "requires DORA_REPLAY_TEST_CLI built from this checkout"]
fn speed_zero_rejects_inputs_lost_at_natural_stop() -> eyre::Result<()> {
    run_test_case(SinkBehavior::Stop)
}

#[test]
#[ignore = "requires DORA_REPLAY_TEST_CLI built from this checkout"]
fn speed_zero_rejects_a_missing_final_message() -> eyre::Result<()> {
    run_test_case(SinkBehavior::MissingSuffix)
}

#[test]
#[ignore = "requires DORA_REPLAY_TEST_CLI built from this checkout"]
fn speed_zero_rejects_a_missing_receipt() -> eyre::Result<()> {
    run_test_case(SinkBehavior::MissingReceipt)
}

#[test]
#[ignore = "requires DORA_REPLAY_TEST_CLI built from this checkout"]
fn speed_zero_times_out_a_stalled_consumer() -> eyre::Result<()> {
    run_test_case(SinkBehavior::Stall)
}

#[test]
#[ignore = "requires DORA_REPLAY_TEST_CLI built from this checkout"]
fn speed_zero_verifies_large_payloads() -> eyre::Result<()> {
    run_test_case(SinkBehavior::LargePayload)
}

#[test]
#[ignore = "requires DORA_REPLAY_TEST_CLI built from this checkout"]
fn speed_zero_verifies_large_metadata() -> eyre::Result<()> {
    run_test_case(SinkBehavior::LargeMetadata)
}

#[test]
#[ignore = "requires DORA_REPLAY_TEST_CLI built from this checkout"]
fn speed_zero_rejects_consumer_byte_saturation() -> eyre::Result<()> {
    run_test_case(SinkBehavior::ByteSaturation)
}

#[test]
#[ignore = "requires DORA_REPLAY_TEST_CLI built from this checkout"]
fn speed_zero_verifies_multiple_outputs_and_receivers() -> eyre::Result<()> {
    run_multi_route_test()
}

#[test]
#[cfg(unix)]
#[ignore = "requires DORA_REPLAY_TEST_CLI built from this checkout"]
fn speed_zero_cancellation_fails_after_complete_delivery() -> eyre::Result<()> {
    run_test_case(SinkBehavior::Cancel)
}

fn run_test_case(behavior: SinkBehavior) -> eyre::Result<()> {
    let temp = tempfile::Builder::new()
        .prefix("dora-replay-delivery-")
        .tempdir()?;
    let root = temp.path();
    let result = run_fixture(root, behavior);
    let keep = std::env::var_os("DORA_REPLAY_KEEP_EVIDENCE").is_some();

    match result {
        Ok(()) if keep => {
            let path = temp.keep();
            eprintln!("replay_delivery: PASS; evidence kept at {}", path.display());
            Ok(())
        }
        Ok(()) => {
            eprintln!(
                "replay_delivery: PASS; {} contract checked",
                behavior.name()
            );
            Ok(())
        }
        Err(error) => {
            let path = temp.keep();
            Err(error.wrap_err(format!("evidence kept at {}", path.display())))
        }
    }
}

fn run_multi_route_test() -> eyre::Result<()> {
    let temp = tempfile::Builder::new()
        .prefix("dora-replay-multi-route-")
        .tempdir()?;
    let result = run_multi_route_fixture(temp.path());
    let keep = std::env::var_os("DORA_REPLAY_KEEP_EVIDENCE").is_some();

    match result {
        Ok(()) if keep => {
            let path = temp.keep();
            eprintln!("replay_delivery: PASS; evidence kept at {}", path.display());
            Ok(())
        }
        Ok(()) => Ok(()),
        Err(error) => {
            let path = temp.keep();
            Err(error.wrap_err(format!("evidence kept at {}", path.display())))
        }
    }
}

fn run_multi_route_fixture(root: &Path) -> eyre::Result<()> {
    let cli = cli_path()?;
    let sink = std::env::current_exe()?;
    let recording = root.join("multi-route.drec");
    let left_result = root.join("left.txt");
    let right_primary_result = root.join("right-primary.txt");
    let right_mirror_result = root.join("right-mirror.txt");
    write_multi_route_recording(
        &recording,
        &sink,
        &left_result,
        &right_primary_result,
        &right_mirror_result,
    )?;

    let mut replay = Command::new(cli);
    replay
        .arg("replay")
        .arg(&recording)
        .args(["--speed", "0", "--delivery-timeout", "30s"])
        .current_dir(root);
    let output = run_command_bounded(&mut replay, root, "replay")?;
    ensure_success("multi-route verified replay", &output)?;

    for (path, input, stream) in [
        (&left_result, "left_input", "left"),
        (&right_primary_result, "right_primary", "right"),
        (&right_mirror_result, "right_mirror", "right"),
    ] {
        let evidence = fs::read_to_string(path)
            .wrap_err_with(|| format!("receiver did not write {}", path.display()))?;
        let expected = format!(
            "input={input};stream={stream};received={MULTI_OUTPUT_COUNT};first=0;last={};valid=true",
            MULTI_OUTPUT_COUNT - 1
        );
        eyre::ensure!(
            evidence.trim() == expected,
            "unexpected receiver evidence in {}: {evidence:?}",
            path.display()
        );
    }
    eyre::ensure!(
        String::from_utf8_lossy(&output.stderr)
            .contains("Verified replay delivery on 3 receiving edges"),
        "CLI did not verify all three receiving edges: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    Ok(())
}

fn run_fixture(root: &Path, behavior: SinkBehavior) -> eyre::Result<()> {
    let cli = cli_path()?;
    let replay_node = cli
        .parent()
        .context("dora CLI has no parent directory")?
        .join(format!("dora-replay-node{}", std::env::consts::EXE_SUFFIX));
    eyre::ensure!(
        replay_node.is_file(),
        "exact replay node is missing at {}",
        replay_node.display()
    );

    let sink = std::env::current_exe()?;
    let recording = root.join("ordered-100.drec");
    let receiver_result = root.join("receiver.txt");
    let receiver_ready = root.join("receiver.ready");
    let receiver_release = root.join("receiver.release");
    let generated = root.join("generated-replay.yml");
    write_recording(
        &recording,
        &sink,
        &receiver_result,
        &receiver_ready,
        &receiver_release,
        behavior,
    )?;
    validate_recording(&recording, behavior)?;

    let mut generate = Command::new(&cli);
    generate
        .arg("replay")
        .arg(&recording)
        .args(["--speed", "0", "--output-yaml"])
        .arg(&generated)
        .current_dir(root);
    let output = run_command_bounded(&mut generate, root, "generate")?;
    ensure_success("replay descriptor generation", &output)?;
    validate_generated_descriptor(&generated, &replay_node, behavior)?;

    let mut replay = Command::new(&cli);
    replay
        .arg("replay")
        .arg(&recording)
        .args(["--speed", "0"])
        .current_dir(root);
    if behavior == SinkBehavior::Stall {
        replay.args(["--delivery-timeout", "2s"]);
    } else if behavior == SinkBehavior::ByteSaturation {
        replay.args(["--delivery-timeout", "60s"]);
    }
    let output = match behavior {
        SinkBehavior::Stop | SinkBehavior::ByteSaturation => run_command_after_gate(
            &mut replay,
            root,
            "replay",
            &receiver_ready,
            &receiver_release,
            if behavior == SinkBehavior::ByteSaturation {
                "verified replay input byte limit reached"
            } else {
                "recorded-source finished successfully"
            },
            if behavior == SinkBehavior::ByteSaturation {
                Duration::from_secs(75)
            } else {
                Duration::from_secs(30)
            },
        )?,
        #[cfg(unix)]
        SinkBehavior::Cancel => run_cancelled_command(&mut replay, root, &receiver_release)?,
        _ => run_command_bounded(&mut replay, root, "replay")?,
    };
    if behavior == SinkBehavior::Stall {
        eyre::ensure!(
            receiver_ready.is_file(),
            "consumer did not start before timeout"
        );
        eyre::ensure!(!output.status.success(), "stalled replay reported success");
        eyre::ensure!(
            String::from_utf8_lossy(&output.stderr)
                .contains("dataflow stopped before verified completion"),
            "unexpected stall failure: {}",
            String::from_utf8_lossy(&output.stderr)
        );
        return Ok(());
    }
    let receiver = fs::read_to_string(&receiver_result).wrap_err_with(|| {
        format!(
            "receiver did not write evidence to {}",
            receiver_result.display()
        )
    })?;
    let exact = receiver.trim()
        == format!(
            "received={};first=0;last={};valid=true",
            behavior.count(),
            behavior.count() - 1
        );
    let stderr = String::from_utf8_lossy(&output.stderr);
    if behavior == SinkBehavior::ByteSaturation {
        eyre::ensure!(
            !output.status.success(),
            "byte-saturated replay reported success: {receiver}"
        );
        eyre::ensure!(
            receiver.trim() == "received=6;first=0;last=5;valid=false",
            "byte-saturation evidence did not show the exact admitted prefix: {receiver}"
        );
        eyre::ensure!(
            stderr.contains("replay delivery incomplete"),
            "unexpected byte-saturation failure: {stderr}"
        );
        let logs = format!("{}\n{}", String::from_utf8_lossy(&output.stdout), stderr);
        eyre::ensure!(
            logs.contains("verified replay input byte limit reached"),
            "replay did not report byte admission failure:\n{logs}"
        );
        return Ok(());
    }
    #[cfg(unix)]
    if behavior == SinkBehavior::Cancel {
        eyre::ensure!(
            exact,
            "cancellation probe did not receive every message: {receiver}"
        );
        eyre::ensure!(
            !output.status.success(),
            "cancelled replay reported success"
        );
        eyre::ensure!(
            stderr.contains("dataflow stopped before verified completion"),
            "unexpected cancellation failure: {stderr}"
        );
        let stdout = String::from_utf8_lossy(&output.stdout);
        eyre::ensure!(
            stdout.contains("sink finished successfully"),
            "consumer failed independently of cancellation: {stdout}"
        );
        return Ok(());
    }
    if behavior.expects_success() || (behavior == SinkBehavior::Stop && exact) {
        ensure_success("verified replay", &output)?;
        eyre::ensure!(exact, "unexpected receiver evidence: {receiver:?}");
        eyre::ensure!(stderr.contains("Verified replay delivery"));
    } else {
        eyre::ensure!(
            !output.status.success(),
            "replay exited successfully despite {}: {receiver:?}",
            behavior.name()
        );
        let expected_error = if behavior == SinkBehavior::MissingReceipt {
            eyre::ensure!(exact, "receipt-loss case must receive every message");
            "did not provide a valid input receipt"
        } else {
            eyre::ensure!(!exact, "loss probe did not lose a message");
            "replay delivery incomplete"
        };
        eyre::ensure!(
            stderr.contains(expected_error),
            "unexpected replay error: {stderr}"
        );
    }
    Ok(())
}

fn write_multi_route_recording(
    path: &Path,
    sink: &Path,
    left_result: &Path,
    right_primary_result: &Path,
    right_mirror_result: &Path,
) -> eyre::Result<()> {
    let receiver = |id: &str, input: &str, stream: &str, result: &Path| {
        serde_json::json!({
            "id": id,
            "path": sink,
            "args": "--ignored --exact replay_multi_sink --nocapture",
            "env": {
                "DORA_REPLAY_MULTI_INPUT": input,
                "DORA_REPLAY_MULTI_STREAM": stream,
                "DORA_REPLAY_MULTI_RESULT": result,
                "DORA_REPLAY_MULTI_COUNT": MULTI_OUTPUT_COUNT.to_string()
            },
            "inputs": {(input): format!("{SOURCE_ID}/{stream}")}
        })
    };
    let descriptor = serde_json::json!({
        "nodes": [
            {"id": SOURCE_ID, "path": sink, "outputs": ["left", "right"]},
            receiver("sink-left", "left_input", "left", left_result),
            receiver("sink-right-primary", "right_primary", "right", right_primary_result),
            receiver("sink-right-mirror", "right_mirror", "right", right_mirror_result)
        ]
    });
    let header = RecordingHeader {
        version: FORMAT_VERSION,
        start_nanos: 0,
        dataflow_id: DataflowId::nil(),
        descriptor_yaml: serde_yaml::to_string(&descriptor)?.into_bytes(),
    };
    let mut writer = RecordingWriter::new(File::create(path)?, &header)?;
    let clock = HLC::default();
    let mut ordinal = 0u64;
    for sequence in 0..MULTI_OUTPUT_COUNT {
        for output in ["left", "right"] {
            writer.write_entry(&multi_route_entry(output, sequence, ordinal, &clock)?)?;
            ordinal += 1;
        }
    }
    let footer = writer.finish()?;
    eyre::ensure!(footer.total_messages == (MULTI_OUTPUT_COUNT * 2) as u64);
    Ok(())
}

fn multi_route_entry(
    output: &str,
    sequence: usize,
    ordinal: u64,
    clock: &HLC,
) -> eyre::Result<RecordEntry> {
    let signature = multi_route_signature(output, sequence);
    let marker = if output == "left" { 11 } else { 29 };
    let encoded =
        encode_arrow_ipc(&Int64Array::from(vec![sequence as i64, signature, marker]).into_arrow())?;
    let timestamp = clock.new_timestamp();
    let metadata = Metadata::from_parameters(
        timestamp,
        BTreeMap::from([
            ("stream".to_string(), Parameter::String(output.to_string())),
            ("sequence".to_string(), Parameter::Integer(sequence as i64)),
            ("signature".to_string(), Parameter::Integer(signature)),
        ]),
    );
    let event = Timestamped {
        inner: InterDaemonEvent::Output {
            dataflow_id: DataflowId::nil(),
            node_id: NodeId::from(SOURCE_ID.to_string()),
            output_id: DataId::from(output.to_string()),
            metadata,
            data: Some(AVec::<u8, ConstAlign<128>>::from_slice(128, &encoded)),
        },
        timestamp,
    };
    Ok(RecordEntry {
        node_id: SOURCE_ID.to_string(),
        output_id: output.to_string(),
        timestamp_offset_nanos: ordinal,
        event_bytes: event.serialize()?,
    })
}

fn write_recording(
    path: &Path,
    sink: &Path,
    receiver_result: &Path,
    receiver_ready: &Path,
    receiver_release: &Path,
    behavior: SinkBehavior,
) -> eyre::Result<()> {
    let inputs = if matches!(behavior, SinkBehavior::Stop | SinkBehavior::ByteSaturation) {
        serde_json::json!({
            "samples": {
                "source": format!("{SOURCE_ID}/{OUTPUT_ID}"),
                "queue_size": if behavior == SinkBehavior::Stop { MESSAGE_COUNT } else { 64 },
                "queue_policy": "backpressure"
            }
        })
    } else {
        serde_json::json!({"samples": format!("{SOURCE_ID}/{OUTPUT_ID}")})
    };
    let descriptor = serde_json::json!({
        "nodes": [
            {"id": SOURCE_ID, "path": sink, "outputs": [OUTPUT_ID]},
            {
                "id": "sink",
                "path": sink,
                "args": "--ignored --exact replay_sink --nocapture",
                "env": {
                    "DORA_REPLAY_TEST_RESULT": receiver_result,
                    "DORA_REPLAY_TEST_COUNT": behavior.count().to_string(),
                    "DORA_REPLAY_TEST_PAYLOAD_ELEMENTS": behavior.payload_elements().to_string(),
                    "DORA_REPLAY_TEST_METADATA_BYTES": behavior.metadata_bytes().to_string(),
                    "DORA_REPLAY_TEST_READY": receiver_ready,
                    "DORA_REPLAY_TEST_RELEASE": receiver_release,
                    "DORA_REPLAY_TEST_BEHAVIOR": behavior.name()
                },
                "inputs": inputs
            }
        ]
    });
    let descriptor_yaml = serde_yaml::to_string(&descriptor)?;
    let clock = HLC::default();
    let header = RecordingHeader {
        version: FORMAT_VERSION,
        start_nanos: 0,
        dataflow_id: DataflowId::nil(),
        descriptor_yaml: descriptor_yaml.into_bytes(),
    };
    let file = File::create(path)?;
    let mut writer = RecordingWriter::new(file, &header)?;

    for sequence in 0..behavior.count() {
        writer.write_entry(&record_entry(sequence, &clock, behavior)?)?;
    }
    let footer = writer.finish()?;
    eyre::ensure!(footer.total_messages == behavior.count() as u64);
    Ok(())
}

fn record_entry(sequence: usize, clock: &HLC, behavior: SinkBehavior) -> eyre::Result<RecordEntry> {
    let signature = content_signature(sequence);
    let mut values = vec![signature; behavior.payload_elements()];
    values[0] = sequence as i64;
    let array = Int64Array::from(values).into_arrow();
    let encoded = encode_arrow_ipc(&array)?;
    let timestamp = clock.new_timestamp();
    let mut parameters = BTreeMap::from([
        (
            "stream".to_string(),
            Parameter::String("ordered-100".to_string()),
        ),
        ("sequence".to_string(), Parameter::Integer(sequence as i64)),
        ("signature".to_string(), Parameter::Integer(signature)),
    ]);
    if behavior.metadata_bytes() > 0 {
        parameters.insert(
            "padding".into(),
            Parameter::String("a".repeat(behavior.metadata_bytes())),
        );
    }
    let event = Timestamped {
        inner: InterDaemonEvent::Output {
            dataflow_id: DataflowId::nil(),
            node_id: NodeId::from(SOURCE_ID.to_string()),
            output_id: DataId::from(OUTPUT_ID.to_string()),
            metadata: Metadata::from_parameters(timestamp, parameters),
            data: Some(AVec::<u8, ConstAlign<128>>::from_slice(128, &encoded)),
        },
        timestamp,
    };
    Ok(RecordEntry {
        node_id: SOURCE_ID.to_string(),
        output_id: OUTPUT_ID.to_string(),
        timestamp_offset_nanos: sequence as u64,
        event_bytes: event.serialize()?,
    })
}

fn validate_recording(path: &Path, behavior: SinkBehavior) -> eyre::Result<()> {
    let mut reader = RecordingReader::open(File::open(path)?)?;
    for expected in 0..behavior.count() {
        let entry = reader.next_entry()?.context("recording ended early")?;
        eyre::ensure!(entry.node_id == SOURCE_ID && entry.output_id == OUTPUT_ID);
        let event = Timestamped::deserialize_inter_daemon_event(&entry.event_bytes)?;
        let InterDaemonEvent::Output { metadata, data, .. } = event.inner else {
            eyre::bail!("record {expected} is not an output")
        };
        let data = data.context("recorded output has no payload")?;
        validate_message(
            expected,
            &metadata,
            decode_arrow_ipc(&data)?,
            behavior.payload_elements(),
            behavior.metadata_bytes(),
        )?;
    }
    eyre::ensure!(
        reader.next_entry()?.is_none(),
        "recording has extra entries"
    );
    Ok(())
}

fn validate_generated_descriptor(
    path: &Path,
    replay_node: &Path,
    behavior: SinkBehavior,
) -> eyre::Result<()> {
    let descriptor: serde_yaml::Value = serde_yaml::from_reader(File::open(path)?)?;
    let nodes = descriptor["nodes"]
        .as_sequence()
        .context("nodes is not a list")?;
    let source = nodes
        .iter()
        .find(|node| node["id"].as_str() == Some(SOURCE_ID))
        .context("generated descriptor has no replay source")?;
    let sink = nodes
        .iter()
        .find(|node| node["id"].as_str() == Some("sink"))
        .context("generated descriptor has no sink")?;
    eyre::ensure!(source["path"].as_str() == replay_node.to_str());
    eyre::ensure!(source["env"]["DORA_REPLAY_SPEED"].as_str() == Some("0"));
    let input = &sink["inputs"]["samples"];
    eyre::ensure!(input["source"].as_str() == Some("recorded-source/samples"));
    let expected_queue_size = if behavior == SinkBehavior::ByteSaturation {
        64
    } else {
        behavior.count().max(10)
    };
    eyre::ensure!(input["queue_size"].as_u64() == Some(expected_queue_size as u64));
    eyre::ensure!(input["queue_policy"].as_str() == Some("backpressure"));
    Ok(())
}

#[test]
#[ignore = "helper process launched as a dataflow node"]
fn replay_sink() -> eyre::Result<()> {
    if std::env::var_os("DORA_NODE_CONFIG").is_none() {
        return Ok(());
    }
    tracing_subscriber::fmt()
        .with_ansi(false)
        .try_init()
        .map_err(|error| eyre::eyre!("failed to initialize sink diagnostics: {error}"))?;
    let expected: usize = std::env::var("DORA_REPLAY_TEST_COUNT")?.parse()?;
    let payload_elements: usize = std::env::var("DORA_REPLAY_TEST_PAYLOAD_ELEMENTS")?.parse()?;
    let metadata_bytes: usize = std::env::var("DORA_REPLAY_TEST_METADATA_BYTES")?.parse()?;
    let result = PathBuf::from(std::env::var("DORA_REPLAY_TEST_RESULT")?);
    let (node, mut events) = DoraNode::init_from_env_force()?;
    let behavior = std::env::var("DORA_REPLAY_TEST_BEHAVIOR")?;
    let break_on_stop = behavior == "stop";
    let ready = PathBuf::from(std::env::var("DORA_REPLAY_TEST_READY")?);
    fs::write(&ready, format!("pid={}", std::process::id()))?;
    if behavior == "stall" {
        loop {
            thread::sleep(Duration::from_secs(1));
        }
    }
    if matches!(behavior.as_str(), "stop" | "byte_saturation") {
        let release = PathBuf::from(std::env::var("DORA_REPLAY_TEST_RELEASE")?);
        wait_for_file(&release, Duration::from_secs(75))?;
    }
    let mut received = 0usize;
    let mut first = None;
    let mut last = None;
    let mut errors = Vec::new();

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, data } => {
                if id.as_str() != "samples" {
                    errors.push(format!("unexpected input `{id}`"));
                    continue;
                }
                match sequence_value(&data) {
                    Ok(sequence) => {
                        first.get_or_insert(sequence);
                        last = Some(sequence);
                    }
                    Err(error) => errors.push(format!("message {received}: {error:#}")),
                }
                if let Err(error) =
                    validate_message(received, &metadata, data, payload_elements, metadata_bytes)
                {
                    errors.push(format!("message {received}: {error:#}"));
                }
                received += 1;
                if behavior == "missing_suffix" && received == expected - 1 {
                    break;
                }
                if behavior == "cancel" && received == expected {
                    break;
                }
            }
            Event::Stop(_) if break_on_stop => break,
            _ => {}
        }
    }
    let valid = received == expected && errors.is_empty();
    fs::write(
        &result,
        format!(
            "received={received};first={};last={};valid={valid}\n{}",
            first.map_or_else(|| "none".to_string(), |value| value.to_string()),
            last.map_or_else(|| "none".to_string(), |value| value.to_string()),
            errors.join("\n")
        ),
    )?;
    eprintln!(
        "replay_delivery sink: received {received} of {expected} ordered messages; valid={valid}"
    );
    if behavior == "cancel" {
        fs::write(result.with_extension("consumed"), "consumed")?;
        wait_for_file(
            &PathBuf::from(std::env::var("DORA_REPLAY_TEST_RELEASE")?),
            Duration::from_secs(15),
        )?;
    }
    drop(events);
    drop(node);
    if behavior == "missing_receipt" {
        let config: dora_recording::replay_receipt::ReplayReceiptConfig =
            serde_json::from_str(&std::env::var("DORA_REPLAY_INPUT_RECEIPT")?)?;
        fs::remove_file(config.receipt_path)?;
    }
    Ok(())
}

#[test]
#[ignore = "helper process launched as a dataflow node"]
fn replay_multi_sink() -> eyre::Result<()> {
    if std::env::var_os("DORA_NODE_CONFIG").is_none() {
        return Ok(());
    }
    let expected: usize = std::env::var("DORA_REPLAY_MULTI_COUNT")?.parse()?;
    let expected_input = std::env::var("DORA_REPLAY_MULTI_INPUT")?;
    let expected_stream = std::env::var("DORA_REPLAY_MULTI_STREAM")?;
    let result = PathBuf::from(std::env::var("DORA_REPLAY_MULTI_RESULT")?);
    let (node, mut events) = DoraNode::init_from_env_force()?;
    let mut received = 0usize;
    let mut first = None;
    let mut last = None;
    let mut errors = Vec::new();

    while let Some(event) = events.recv() {
        if let Event::Input { id, metadata, data } = event {
            if id.as_str() != expected_input {
                errors.push(format!("unexpected input `{id}`"));
                continue;
            }
            match validate_multi_route_message(received, &expected_stream, &metadata, data) {
                Ok(sequence) => {
                    first.get_or_insert(sequence);
                    last = Some(sequence);
                }
                Err(error) => errors.push(format!("message {received}: {error:#}")),
            }
            received += 1;
        }
    }
    let valid = received == expected && errors.is_empty();
    fs::write(
        &result,
        format!(
            "input={expected_input};stream={expected_stream};received={received};first={};last={};valid={valid}\n{}",
            first.map_or_else(|| "none".to_string(), |value| value.to_string()),
            last.map_or_else(|| "none".to_string(), |value| value.to_string()),
            errors.join("\n")
        ),
    )?;
    drop(events);
    drop(node);
    Ok(())
}

fn validate_multi_route_message(
    expected: usize,
    stream: &str,
    metadata: &Metadata,
    data: dora_node_api::DoraArray,
) -> eyre::Result<i64> {
    let array = data
        .as_array()
        .as_any()
        .downcast_ref::<Int64Array>()
        .context("payload is not Int64Array")?;
    let signature = multi_route_signature(stream, expected);
    let marker = if stream == "left" { 11 } else { 29 };
    eyre::ensure!(
        array.len() == 3
            && array.value(0) == expected as i64
            && array.value(1) == signature
            && array.value(2) == marker,
        "payload content or order changed"
    );
    eyre::ensure!(
        metadata.parameters.get("stream") == Some(&Parameter::String(stream.to_string())),
        "stream metadata changed"
    );
    eyre::ensure!(
        metadata.parameters.get("sequence") == Some(&Parameter::Integer(expected as i64)),
        "sequence metadata changed"
    );
    eyre::ensure!(
        metadata.parameters.get("signature") == Some(&Parameter::Integer(signature)),
        "signature metadata changed"
    );
    Ok(array.value(0))
}

fn sequence_value(data: &dora_node_api::DoraArray) -> eyre::Result<i64> {
    let array = data
        .as_array()
        .as_any()
        .downcast_ref::<Int64Array>()
        .context("payload is not Int64Array")?;
    eyre::ensure!(!array.is_empty(), "payload is empty");
    Ok(array.value(0))
}

fn validate_message(
    expected: usize,
    metadata: &Metadata,
    data: dora_node_api::DoraArray,
    payload_elements: usize,
    metadata_bytes: usize,
) -> eyre::Result<()> {
    let array = data
        .as_array()
        .as_any()
        .downcast_ref::<Int64Array>()
        .context("payload is not Int64Array")?;
    eyre::ensure!(
        array.len() == payload_elements,
        "message {expected} has wrong length"
    );
    let signature = content_signature(expected);
    eyre::ensure!(array.value(0) == expected as i64, "payload is out of order");
    eyre::ensure!(
        array.iter().enumerate().all(|(index, value)| {
            value
                == Some(if index == 0 {
                    expected as i64
                } else {
                    signature
                })
        }),
        "payload content changed"
    );
    if metadata_bytes > 0 {
        let Some(Parameter::String(padding)) = metadata.parameters.get("padding") else {
            eyre::bail!("large metadata is missing");
        };
        eyre::ensure!(
            padding.len() == metadata_bytes && padding.bytes().all(|byte| byte == b'a'),
            "metadata content changed"
        );
    }
    eyre::ensure!(
        metadata.parameters.get("stream") == Some(&Parameter::String("ordered-100".to_string()))
    );
    eyre::ensure!(
        metadata.parameters.get("sequence") == Some(&Parameter::Integer(expected as i64))
    );
    eyre::ensure!(metadata.parameters.get("signature") == Some(&Parameter::Integer(signature)));
    Ok(())
}

fn content_signature(sequence: usize) -> i64 {
    (sequence as i64 * 7_919) ^ 0x5a5a_1357
}

fn multi_route_signature(stream: &str, sequence: usize) -> i64 {
    let stream_marker = if stream == "left" { 0x11 } else { 0x29 };
    content_signature(sequence) ^ stream_marker
}

fn cli_path() -> eyre::Result<PathBuf> {
    let path = std::env::var_os("DORA_REPLAY_TEST_CLI")
        .map(PathBuf::from)
        .context("set DORA_REPLAY_TEST_CLI to the exact built dora CLI")?;
    eyre::ensure!(path.is_file(), "dora CLI is missing at {}", path.display());
    Ok(path)
}

struct CapturedOutput {
    status: ExitStatus,
    stdout: Vec<u8>,
    stderr: Vec<u8>,
}

fn run_command_bounded(
    command: &mut Command,
    root: &Path,
    name: &str,
) -> eyre::Result<CapturedOutput> {
    let stdout_path = root.join(format!("{name}.stdout.log"));
    let stderr_path = root.join(format!("{name}.stderr.log"));
    command
        .stdout(Stdio::from(File::create(&stdout_path)?))
        .stderr(Stdio::from(File::create(&stderr_path)?));
    let mut child = spawn_owned(command, root, name)?;
    let status = wait_child_bounded(&mut child, name, Duration::from_secs(30))?;
    Ok(CapturedOutput {
        status,
        stdout: fs::read(stdout_path)?,
        stderr: fs::read(stderr_path)?,
    })
}

fn run_command_after_gate(
    command: &mut Command,
    root: &Path,
    name: &str,
    ready: &Path,
    release: &Path,
    gate_text: &str,
    timeout: Duration,
) -> eyre::Result<CapturedOutput> {
    let stdout_path = root.join(format!("{name}.stdout.log"));
    let stderr_path = root.join(format!("{name}.stderr.log"));
    command
        .stdout(Stdio::from(File::create(&stdout_path)?))
        .stderr(Stdio::from(File::create(&stderr_path)?));
    let mut child = spawn_owned(command, root, name)?;
    wait_for_file(ready, timeout)?;
    wait_for_text(&stdout_path, gate_text, timeout)?;
    fs::write(release, "release")?;
    let status = wait_child_bounded(&mut child, name, timeout)?;
    Ok(CapturedOutput {
        status,
        stdout: fs::read(stdout_path)?,
        stderr: fs::read(stderr_path)?,
    })
}

#[cfg(unix)]
fn run_cancelled_command(
    command: &mut Command,
    root: &Path,
    release: &Path,
) -> eyre::Result<CapturedOutput> {
    let stdout_path = root.join("replay.stdout.log");
    let stderr_path = root.join("replay.stderr.log");
    command
        .stdout(Stdio::from(File::create(&stdout_path)?))
        .stderr(Stdio::from(File::create(&stderr_path)?));
    let mut child = spawn_owned(command, root, "replay")?;
    wait_for_file(&root.join("receiver.consumed"), Duration::from_secs(15))?;
    eyre::ensure!(signal_process_group(child.child.id(), "-INT")?.success());
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        let stopped = [&stdout_path, &stderr_path].iter().any(|path| {
            fs::read_to_string(path).is_ok_and(|text| text.contains("received ctrlc signal"))
        });
        if stopped {
            break;
        }
        eyre::ensure!(
            Instant::now() < deadline,
            "daemon did not observe cancellation"
        );
        thread::sleep(Duration::from_millis(10));
    }
    fs::write(release, "release")?;
    let status = wait_child_bounded(&mut child, "cancelled replay", Duration::from_secs(30))?;
    Ok(CapturedOutput {
        status,
        stdout: fs::read(stdout_path)?,
        stderr: fs::read(stderr_path)?,
    })
}

fn wait_child_bounded(
    child: &mut OwnedChild,
    name: &str,
    timeout: Duration,
) -> eyre::Result<ExitStatus> {
    let deadline = Instant::now() + timeout;
    let status = loop {
        if let Some(status) = child.child.try_wait()? {
            child.completed = true;
            break status;
        }
        if Instant::now() >= deadline {
            child.kill_tree();
            child.child.wait()?;
            child.completed = true;
            eyre::bail!("{name} timed out after {} seconds", timeout.as_secs());
        }
        thread::sleep(Duration::from_millis(20));
    };
    fs::write(
        child.root.join(format!("{name}.elapsed-seconds.txt")),
        format!("{:.6}\n", child.started.elapsed().as_secs_f64()),
    )?;
    Ok(status)
}

struct OwnedChild {
    child: std::process::Child,
    root: PathBuf,
    completed: bool,
    started: Instant,
}

fn spawn_owned(command: &mut Command, root: &Path, name: &str) -> eyre::Result<OwnedChild> {
    #[cfg(unix)]
    command.process_group(0);
    Ok(OwnedChild {
        child: command
            .spawn()
            .wrap_err_with(|| format!("failed to start {name}"))?,
        root: root.to_owned(),
        completed: false,
        started: Instant::now(),
    })
}

impl OwnedChild {
    fn kill_tree(&mut self) {
        #[cfg(unix)]
        {
            let owned_nodes = capture_owned_node_processes(&self.root, self.child.id());
            let _ = signal_process_group(self.child.id(), "-INT");
            let deadline = Instant::now() + Duration::from_secs(2);
            let mut cli_exited = false;
            while Instant::now() < deadline {
                if !cli_exited && self.child.try_wait().ok().flatten().is_some() {
                    self.completed = true;
                    cli_exited = true;
                }
                if cli_exited && owned_nodes.iter().all(|process| !process.matches()) {
                    return;
                }
                thread::sleep(Duration::from_millis(20));
            }
            for process in owned_nodes {
                if process.matches() {
                    let _ = signal_process_group(process.pid, "-KILL");
                }
            }
            if !cli_exited {
                let _ = signal_process_group(self.child.id(), "-KILL");
            }
        }
        #[cfg(not(unix))]
        {
            let _ = self.child.kill();
        }
    }
}

#[cfg(unix)]
#[derive(PartialEq)]
struct ProcessIdentity {
    start: String,
    command: String,
}

#[cfg(unix)]
struct OwnedNodeProcess {
    pid: u32,
    identity: ProcessIdentity,
}

#[cfg(unix)]
impl OwnedNodeProcess {
    fn matches(&self) -> bool {
        process_identity(self.pid).as_ref() == Some(&self.identity)
    }
}

#[cfg(unix)]
fn capture_owned_node_processes(root: &Path, parent: u32) -> Vec<OwnedNodeProcess> {
    fixture_node_process_groups(root)
        .into_iter()
        .filter(|pid| has_parent(*pid, parent))
        .filter_map(|pid| process_identity(pid).map(|identity| OwnedNodeProcess { pid, identity }))
        .collect()
}

#[cfg(unix)]
fn process_identity(pid: u32) -> Option<ProcessIdentity> {
    Some(ProcessIdentity {
        start: process_field(pid, "lstart=")?,
        command: process_field(pid, "command=")?,
    })
}

#[cfg(unix)]
fn process_field(pid: u32, field: &str) -> Option<String> {
    let pid = pid.to_string();
    Command::new("ps")
        .args(["-p", &pid, "-o", field])
        .output()
        .ok()
        .filter(|output| output.status.success())
        .and_then(|output| String::from_utf8(output.stdout).ok())
        .map(|value| value.trim().to_string())
        .filter(|value| !value.is_empty())
}

#[cfg(unix)]
fn has_parent(pid: u32, parent: u32) -> bool {
    Command::new("ps")
        .args(["-p", &pid.to_string(), "-o", "ppid="])
        .output()
        .ok()
        .filter(|output| output.status.success())
        .and_then(|output| String::from_utf8(output.stdout).ok())
        .and_then(|value| value.trim().parse::<u32>().ok())
        == Some(parent)
}

#[cfg(unix)]
fn signal_process_group(pid: u32, signal: &str) -> std::io::Result<ExitStatus> {
    Command::new("/bin/kill")
        .arg(signal)
        .arg(format!("-{pid}"))
        .status()
}

#[cfg(unix)]
fn fixture_node_process_groups(root: &Path) -> Vec<u32> {
    let mut pids = Vec::new();
    if let Ok(ready) = fs::read_to_string(root.join("receiver.ready"))
        && let Some(pid) = ready
            .strip_prefix("pid=")
            .and_then(|value| value.parse().ok())
    {
        pids.push(pid);
    }
    if let Ok(log) = fs::read_to_string(root.join("replay.stdout.log")) {
        for line in log
            .lines()
            .filter(|line| line.contains("spawned node with pid "))
        {
            if let Some(pid) = line
                .rsplit_once("spawned node with pid ")
                .and_then(|(_, value)| value.parse().ok())
                && !pids.contains(&pid)
            {
                pids.push(pid);
            }
        }
    }
    pids
}

impl Drop for OwnedChild {
    fn drop(&mut self) {
        if !self.completed {
            self.kill_tree();
            let _ = self.child.wait();
        }
    }
}

fn wait_for_file(path: &Path, timeout: Duration) -> eyre::Result<()> {
    let deadline = Instant::now() + timeout;
    while !path.is_file() {
        eyre::ensure!(
            Instant::now() < deadline,
            "timed out waiting for {}",
            path.display()
        );
        thread::sleep(Duration::from_millis(10));
    }
    Ok(())
}

fn wait_for_text(path: &Path, expected: &str, timeout: Duration) -> eyre::Result<()> {
    let deadline = Instant::now() + timeout;
    loop {
        if fs::read_to_string(path)
            .map(|text| text.contains(expected))
            .unwrap_or(false)
        {
            return Ok(());
        }
        eyre::ensure!(
            Instant::now() < deadline,
            "timed out waiting for `{expected}`"
        );
        thread::sleep(Duration::from_millis(10));
    }
}

fn ensure_success(step: &str, output: &CapturedOutput) -> eyre::Result<()> {
    eyre::ensure!(
        output.status.success(),
        "{step} failed with {:?}\nstdout:\n{}\nstderr:\n{}",
        output.status,
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
    Ok(())
}
