use std::{
    fs::File,
    io::Write as _,
    path::PathBuf,
    time::{Duration, Instant},
};

use arrow::array::RecordBatch;
use arrow_integration_test::{ArrowJsonField, data_type_to_json};
use colored::Colorize;
use dora_core::{
    metadata::ArrowTypeInfoExt,
    uhlc::{self, HLC, NTP64, Timestamp},
};
use dora_message::{
    common::{DataMessage, Timestamped},
    daemon_to_node::{DaemonReply, NodeEvent},
    integration_testing::{InputDataFormat, InputEvent, IntegrationTestInput, TimedInputEvent},
    metadata::{ArrowTypeInfo, Metadata},
    node_to_daemon::DaemonRequest,
};
use eyre::Context;

use crate::{
    arrow_utils::{copy_array_into_sample, required_data_size},
    daemon_connection::json_to_arrow::{convert_arrow_json_data, read_json_value_as_arrow},
    event_stream::data_to_arrow_array,
};

pub struct IntegrationTestingEvents {
    events: std::vec::IntoIter<TimedInputEvent>,
    output_file: File,
    start_timestamp: uhlc::Timestamp,
    start_time: Instant,
    skip_output_time_offsets: bool,
}

impl IntegrationTestingEvents {
    pub fn new(
        input_file_path: PathBuf,
        output_file_path: PathBuf,
        skip_output_time_offsets: bool,
    ) -> eyre::Result<Self> {
        let mut node_info: IntegrationTestInput = serde_json::from_slice(
            &std::fs::read(&input_file_path)
                .with_context(|| format!("failed to open {}", input_file_path.display()))?,
        )
        .with_context(|| format!("failed to deserialize {}", input_file_path.display()))?;
        let output_file = File::create(&output_file_path)
            .with_context(|| format!("failed to create {}", output_file_path.display()))?;

        node_info
            .events
            .as_mut_slice()
            .sort_by(|a, b| a.time_offset_secs.total_cmp(&b.time_offset_secs));
        let inputs = std::mem::take(&mut node_info.events).into_iter();

        let clock = HLC::default();
        let start_timestamp = clock.new_timestamp();
        let start_time = Instant::now();
        Ok(Self {
            events: inputs,
            output_file,
            start_timestamp,
            start_time,
            skip_output_time_offsets,
        })
    }

    pub fn request(&mut self, request: &Timestamped<DaemonRequest>) -> eyre::Result<DaemonReply> {
        let reply = match &request.inner {
            DaemonRequest::Register(_) => DaemonReply::Result(Ok(())),
            DaemonRequest::Subscribe => DaemonReply::Result(Ok(())),
            DaemonRequest::SubscribeDrop => DaemonReply::Result(Ok(())),
            DaemonRequest::NextEvent { .. } => {
                let events = if let Some(event) = self.next_event()? {
                    vec![event]
                } else {
                    vec![]
                };
                DaemonReply::NextEvents(events)
            }
            DaemonRequest::SendMessage {
                output_id,
                metadata,
                data,
            } => {
                let mut output = serde_json::Map::new();
                output.insert("id".into(), output_id.to_string().into());

                if !self.skip_output_time_offsets {
                    let time_offset = metadata
                        .timestamp()
                        .get_diff_duration(&self.start_timestamp);
                    output.insert("time_offset_secs".into(), time_offset.as_secs_f64().into());
                }

                if data.is_some() {
                    let (drop_tx, drop_rx) = flume::unbounded();
                    let data_array = data_to_arrow_array(data.clone(), metadata, drop_tx)
                        .context("failed to convert output to arrow array")?;
                    // integration testing doesn't use shared memory -> no drop tokens
                    let _ = drop_rx;

                    let data_type_json = data_type_to_json(data_array.data_type());

                    let batch = RecordBatch::try_from_iter([("inner", data_array)])
                        .context("failed to create RecordBatch")?;

                    let mut writer = arrow_json::ArrayWriter::new(Vec::new());
                    writer
                        .write(&batch)
                        .context("failed to encode data as JSON")?;
                    writer
                        .finish()
                        .context("failed to finish writing JSON data")?;
                    let json_data_encoded = writer.into_inner();

                    // Reparse the string using serde_json
                    let json_data: Vec<serde_json::Map<String, serde_json::Value>> =
                        serde_json::from_reader(json_data_encoded.as_slice())
                            .context("failed to parse JSON data again")?;
                    // remove `inner` field again
                    let json_data_flattened: Vec<_> = json_data
                        .into_iter()
                        .map(|mut m| m.remove("inner"))
                        .collect();
                    output.insert("data".into(), json_data_flattened.into());
                    output.insert("type".into(), data_type_json);
                }

                serde_json::to_writer(&mut self.output_file, &output)
                    .context("failed to write output as JSON")?;
                writeln!(&mut self.output_file)
                    .context("failed to write newline to output file")?;

                DaemonReply::Empty
            }
            DaemonRequest::CloseOutputs(data_ids) => {
                println!("{} {data_ids:?}", "node reports closed outputs".blue());
                DaemonReply::Result(Ok(()))
            }
            DaemonRequest::OutputsDone => {
                println!("{}", "node reports OutputsDone".blue());
                DaemonReply::Result(Ok(()))
            }
            DaemonRequest::ReportDropTokens { drop_tokens } => {
                println!("{} {drop_tokens:?}", "node reports drop tokens".blue());
                DaemonReply::Empty
            }
            DaemonRequest::NextFinishedDropTokens => {
                // interactive nodes don't use shared memory -> no drop tokens
                DaemonReply::NextDropEvents(vec![])
            }
            DaemonRequest::EventStreamDropped => {
                println!("{}", "node reports EventStreamDropped".blue());
                DaemonReply::Result(Ok(()))
            }
            DaemonRequest::NodeConfig { .. } => {
                eyre::bail!("unexpected NodeConfig in interactive mode")
            }
        };
        Ok(reply)
    }

    fn next_event(&mut self) -> eyre::Result<Option<Timestamped<NodeEvent>>> {
        let Some(event) = self.events.next() else {
            return Ok(None);
        };

        let TimedInputEvent {
            time_offset_secs,
            event,
        } = event;
        let time_offset = Duration::from_secs_f64(time_offset_secs);
        let elapsed = self.start_time.elapsed();
        if let Some(wait_time) = time_offset.checked_sub(elapsed) {
            std::thread::sleep(wait_time);
        }

        let timestamp = Timestamp::new(
            self.start_timestamp.get_time() + NTP64::from(time_offset),
            *self.start_timestamp.get_id(),
        );

        let converted = match event {
            InputEvent::Stop => NodeEvent::Stop,
            InputEvent::Input {
                id,
                metadata,
                data,
                data_format,
            } => {
                let (data, type_info) = if let Some(data) = data {
                    let array = match data_format {
                        InputDataFormat::JsonObject => {
                            // input is JSON data
                            read_json_value_as_arrow(&data)
                        }
                        InputDataFormat::ArrowTest => convert_arrow_json_data(data, false),
                        InputDataFormat::ArrowTestUnwrap => convert_arrow_json_data(data, true),
                    }
                    .context("failed to read data")?;

                    let total_len = required_data_size(&array);
                    let mut buf = vec![0; total_len];
                    let type_info = copy_array_into_sample(buf.as_mut_slice(), &array);

                    (Some(buf), type_info)
                } else {
                    (None, ArrowTypeInfo::empty())
                };
                let mut meta = Metadata::new(timestamp, type_info);
                meta.parameters = metadata.unwrap_or_default();
                NodeEvent::Input {
                    id,
                    metadata: meta,
                    data: data.map(|d| DataMessage::Vec(aligned_vec::AVec::from_slice(1, &d))),
                }
            }
            InputEvent::InputClosed { id } => NodeEvent::InputClosed { id },
            InputEvent::AllInputsClosed => NodeEvent::AllInputsClosed,
        };
        Ok(Some(Timestamped {
            inner: converted,
            timestamp,
        }))
    }
}
