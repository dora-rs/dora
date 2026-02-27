use std::{collections::HashMap, fs::File, time::SystemTime};

use adora_message::{
    common::Timestamped,
    daemon_to_daemon::InterDaemonEvent,
    id::{DataId, NodeId},
};
use adora_node_api::{AdoraNode, Event, arrow_utils};
use adora_recording::{RecordEntry, RecordingHeader, RecordingWriter};
use aligned_vec::AVec;
use eyre::Context;

fn main() -> eyre::Result<()> {
    let output_file =
        std::env::var("ADORA_RECORD_FILE").wrap_err("ADORA_RECORD_FILE env var not set")?;
    let topics_json =
        std::env::var("ADORA_RECORD_TOPICS").wrap_err("ADORA_RECORD_TOPICS env var not set")?;
    let descriptor_yaml = std::env::var("ADORA_RECORD_DESCRIPTOR").unwrap_or_default();

    // Parse topic map: { "input_id_on_record_node": "source_node/source_output" }
    let topic_map: HashMap<String, String> =
        serde_json::from_str(&topics_json).wrap_err("failed to parse ADORA_RECORD_TOPICS")?;

    // Build reverse map: input_id -> (source_node_id, source_output_id)
    let mut reverse_map: HashMap<String, (String, String)> = HashMap::new();
    for (input_id, source) in &topic_map {
        let (node_id, output_id) = source
            .split_once('/')
            .ok_or_else(|| eyre::eyre!("invalid topic format: {source}"))?;
        reverse_map.insert(
            input_id.clone(),
            (node_id.to_string(), output_id.to_string()),
        );
    }

    let (_node, mut events) = AdoraNode::init_from_env()?;

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

    eprintln!("adora-record-node: recording to {output_file}");

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, data } => {
                let (source_node, source_output) = match reverse_map.get(&*id) {
                    Some(pair) => pair,
                    None => continue,
                };

                let arrow_data = data.to_data();
                let data_size = arrow_utils::required_data_size(&arrow_data);
                let raw_data = if data_size > 0 {
                    let mut buf = vec![0u8; data_size];
                    arrow_utils::copy_array_into_sample(&mut buf, &arrow_data);
                    Some(AVec::from_slice(128, &buf))
                } else {
                    None
                };

                let timestamp = metadata.timestamp();
                let inter_event = InterDaemonEvent::Output {
                    dataflow_id: uuid::Uuid::nil(),
                    node_id: NodeId::from(source_node.clone()),
                    output_id: DataId::from(source_output.clone()),
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
                    node_id: source_node.clone(),
                    output_id: source_output.clone(),
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
    eprintln!("adora-record-node: recording complete");
    eprintln!("  Messages: {msg_count}");
    eprintln!("  Bytes:    {}", footer.total_bytes);
    eprintln!("  File:     {output_file}");

    Ok(())
}
