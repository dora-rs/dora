use std::{collections::HashMap, fs::File, time::SystemTime};

use aligned_vec::AVec;
use dora_message::{
    common::Timestamped,
    daemon_to_daemon::InterDaemonEvent,
    id::{DataId, NodeId},
};
use dora_node_api::{DoraNode, Event, arrow::datatypes::DataType, arrow_utils};
use dora_recording::{RecordEntry, RecordingHeader, RecordingWriter};
use eyre::Context;

/// Parse `DORA_RECORD_TOPICS` (`{ "input_id": "source_node/source_output" }`)
/// into a lookup from the record node's own input id to the validated
/// `(source_node, source_output)` ids.
///
/// The node and output ids are parsed via [`str::parse`] (`FromStr`), which
/// *validates* the character set and returns an error, rather than the
/// panicking `NodeId::from`/`DataId::from` (`From<String>`) conversions. A
/// malformed topic (e.g. a source node id containing a space) therefore fails
/// the node cleanly at startup instead of panicking mid-run when the offending
/// event is first recorded.
fn build_reverse_map(topics_json: &str) -> eyre::Result<HashMap<String, (NodeId, DataId)>> {
    let topic_map: HashMap<String, String> =
        serde_json::from_str(topics_json).wrap_err("failed to parse DORA_RECORD_TOPICS")?;

    let mut reverse_map: HashMap<String, (NodeId, DataId)> = HashMap::new();
    for (input_id, source) in &topic_map {
        let (node_id, output_id) = source
            .split_once('/')
            .ok_or_else(|| eyre::eyre!("invalid topic format: {source}"))?;
        let node_id: NodeId = node_id
            .parse()
            .wrap_err_with(|| format!("invalid source node id in topic `{source}`"))?;
        let output_id: DataId = output_id
            .parse()
            .wrap_err_with(|| format!("invalid source output id in topic `{source}`"))?;
        reverse_map.insert(input_id.clone(), (node_id, output_id));
    }
    Ok(reverse_map)
}

fn main() -> eyre::Result<()> {
    let output_file =
        std::env::var("DORA_RECORD_FILE").wrap_err("DORA_RECORD_FILE env var not set")?;
    let topics_json =
        std::env::var("DORA_RECORD_TOPICS").wrap_err("DORA_RECORD_TOPICS env var not set")?;
    let descriptor_yaml = std::env::var("DORA_RECORD_DESCRIPTOR").unwrap_or_default();

    // Build reverse map: input_id -> (source_node_id, source_output_id).
    let reverse_map = build_reverse_map(&topics_json)?;

    let (_node, mut events) = DoraNode::init_from_env()?;

    let start_nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos() as u64;

    let header = RecordingHeader {
        version: 1,
        start_nanos,
        dataflow_id: uuid::Uuid::new_v4(),
        descriptor_yaml: descriptor_yaml.into_bytes(),
    };

    let file =
        File::create(&output_file).wrap_err_with(|| format!("failed to create {output_file}"))?;
    let mut writer = RecordingWriter::new(file, &header)?;
    let mut msg_count: u64 = 0;

    eprintln!("dora-record-node: recording to {output_file}");

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, data } => {
                let (source_node, source_output) = match reverse_map.get(&*id) {
                    Some(pair) => pair,
                    None => continue,
                };

                // Record the payload as a self-describing Arrow IPC stream so
                // replay can reconstruct the array without a type sidecar.
                let arrow_data = data.to_data();
                let raw_data =
                    if matches!(arrow_data.data_type(), DataType::Null) && arrow_data.is_empty() {
                        // Exactly the unit array that replay rebuilds from an absent
                        // payload (`NullArray::new(0)`) — record `None` to skip the
                        // IPC framing.
                        None
                    } else {
                        // Encode every other array — including a zero-length *typed*
                        // array (e.g. an empty `Float32Array`) and a non-empty
                        // `NullArray` — as a self-describing IPC stream, so replay
                        // preserves the declared type and length instead of
                        // collapsing it to `NullArray::new(0)`. (The deleted
                        // `type_info` sidecar used to preserve this; #2027/#2083.)
                        let ipc_bytes = arrow_utils::encode_arrow_ipc(&arrow_data)
                            .wrap_err("failed to Arrow-IPC-encode recorded output")?;
                        Some(AVec::from_slice(128, &ipc_bytes))
                    };

                let timestamp = metadata.timestamp();
                let inter_event = InterDaemonEvent::Output {
                    dataflow_id: uuid::Uuid::nil(),
                    node_id: source_node.clone(),
                    output_id: source_output.clone(),
                    metadata,
                    data: raw_data,
                };

                let timestamped = Timestamped {
                    inner: inter_event,
                    timestamp,
                };
                let event_bytes = timestamped.serialize()?;

                let now_nanos = SystemTime::now()
                    .duration_since(SystemTime::UNIX_EPOCH)
                    .unwrap()
                    .as_nanos() as u64;

                let entry = RecordEntry {
                    node_id: source_node.to_string(),
                    output_id: source_output.to_string(),
                    timestamp_offset_nanos: now_nanos.saturating_sub(start_nanos),
                    event_bytes,
                };
                writer.write_entry(&entry)?;
                msg_count += 1;
            }
            Event::Stop(_) => break,
            _ => {}
        }
    }

    let footer = writer.finish()?;
    eprintln!("dora-record-node: recording complete");
    eprintln!("  Messages: {msg_count}");
    eprintln!("  Bytes:    {}", footer.total_bytes);
    eprintln!("  File:     {output_file}");

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::build_reverse_map;

    #[test]
    fn valid_topics_parse() {
        let map = build_reverse_map(r#"{"in":"camera/image"}"#).unwrap();
        let (node, output) = map.get("in").unwrap();
        assert_eq!(node.to_string(), "camera");
        assert_eq!(output.to_string(), "image");
    }

    #[test]
    fn missing_slash_is_an_error() {
        assert!(build_reverse_map(r#"{"in":"camera"}"#).is_err());
    }

    // A source node id with an invalid character (a space) must yield a clean
    // startup error, not a panic from the `NodeId::from(String)` conversion
    // that the recorder used to call in its hot loop.
    #[test]
    fn invalid_node_id_is_an_error_not_a_panic() {
        let err = build_reverse_map(r#"{"in":"my node/image"}"#).unwrap_err();
        assert!(
            format!("{err:#}").contains("invalid source node id"),
            "unexpected error: {err:#}"
        );
    }

    #[test]
    fn invalid_output_id_is_an_error_not_a_panic() {
        let err = build_reverse_map(r#"{"in":"camera/bad output"}"#).unwrap_err();
        assert!(
            format!("{err:#}").contains("invalid source output id"),
            "unexpected error: {err:#}"
        );
    }
}
