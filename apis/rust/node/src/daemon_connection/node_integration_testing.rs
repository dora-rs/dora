use std::{
    fs::File,
    io::Write,
    sync::Arc,
    time::{Duration, Instant},
};

use arrow::array::{Array, RecordBatch, StructArray};
use arrow_schema::{DataType, Field, Schema};
use colored::Colorize;
use dora_core::{
    metadata::ArrowTypeInfoExt,
    uhlc::{self, HLC, NTP64, Timestamp},
};
use dora_message::{
    common::{DataMessage, Timestamped},
    daemon_to_node::{DaemonReply, NodeEvent},
    integration_testing_format::{
        IncomingEvent, InputData, IntegrationTestInput, TimedIncomingEvent,
    },
    metadata::{ArrowTypeInfo, Metadata},
    node_to_daemon::DaemonRequest,
};
use eyre::{Context, ContextCompat};

use crate::{
    arrow_utils::{copy_array_into_sample, required_data_size},
    daemon_connection::json_to_arrow::read_json_value_as_arrow,
    event_stream::data_to_arrow_array,
    integration_testing::{TestingInput, TestingOptions, TestingOutput},
};

pub struct IntegrationTestingEvents {
    events: std::vec::IntoIter<TimedIncomingEvent>,
    output_writer: OutputWriter,
    start_timestamp: uhlc::Timestamp,
    start_time: Instant,
    options: TestingOptions,
}

impl IntegrationTestingEvents {
    pub fn new(
        input: TestingInput,
        output: TestingOutput,
        options: TestingOptions,
    ) -> eyre::Result<Self> {
        let mut node_info: IntegrationTestInput = match input {
            TestingInput::FromJsonFile(input_file_path) => serde_json::from_slice(
                &std::fs::read(&input_file_path)
                    .with_context(|| format!("failed to open {}", input_file_path.display()))?,
            )
            .with_context(|| format!("failed to deserialize {}", input_file_path.display()))?,
            TestingInput::Input(input) => input,
        };

        let output_writer = match output {
            TestingOutput::ToFile(output_file_path) => {
                let file = File::create(&output_file_path)
                    .with_context(|| format!("failed to create {}", output_file_path.display()))?;
                OutputWriter::Writer(Box::new(file))
            }
            TestingOutput::ToWriter(writer) => OutputWriter::Writer(writer),
            TestingOutput::ToChannel(sender) => OutputWriter::Channel(sender),
        };

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
            output_writer,
            start_timestamp,
            start_time,
            options,
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
            } => self.handle_output(output_id, metadata, data)?,
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

    fn handle_output(
        &mut self,
        output_id: &dora_message::id::DataId,
        metadata: &Metadata,
        data: &Option<DataMessage>,
    ) -> Result<DaemonReply, eyre::Error> {
        let start_timestamp = self.start_timestamp;
        let skip_output_time_offsets = self.options.skip_output_time_offsets;

        let output = convert_output_to_json(
            output_id,
            metadata,
            data,
            start_timestamp,
            skip_output_time_offsets,
        )?;
        match &mut self.output_writer {
            OutputWriter::Writer(writer) => {
                serde_json::to_writer(writer.as_mut(), &output)
                    .context("failed to write output as JSON")?;
                writeln!(writer.as_mut()).context("failed to write newline to output file")?;
            }
            OutputWriter::Channel(sender) => {
                sender
                    .send(output)
                    .context("failed to send output to channel")?;
            }
        }
        Ok(DaemonReply::Empty)
    }

    fn next_event(&mut self) -> eyre::Result<Option<Timestamped<NodeEvent>>> {
        let Some(event) = self.events.next() else {
            return Ok(None);
        };

        let TimedIncomingEvent {
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
            IncomingEvent::Stop => NodeEvent::Stop,
            IncomingEvent::Input { id, metadata, data } => {
                let (data, type_info) = if let Some(data) = data {
                    let array = read_input_data(*data).with_context(|| {
                        format!("failed to read input event at offset {time_offset_secs}s ")
                    })?;

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
            IncomingEvent::InputClosed { id } => NodeEvent::InputClosed { id },
            IncomingEvent::AllInputsClosed => NodeEvent::AllInputsClosed,
        };
        Ok(Some(Timestamped {
            inner: converted,
            timestamp,
        }))
    }
}

enum OutputWriter {
    Writer(Box<dyn Write + Send>),
    Channel(flume::Sender<serde_json::Map<String, serde_json::Value>>),
}

pub fn convert_output_to_json(
    output_id: &dora_message::id::DataId,
    metadata: &Metadata,
    data: &Option<DataMessage>,
    start_timestamp: Timestamp,
    skip_output_time_offsets: bool,
) -> eyre::Result<serde_json::Map<String, serde_json::Value>> {
    let mut output = serde_json::Map::new();
    output.insert("id".into(), output_id.to_string().into());
    if !skip_output_time_offsets {
        let time_offset = metadata.timestamp().get_diff_duration(&start_timestamp);
        output.insert("time_offset_secs".into(), time_offset.as_secs_f64().into());
    }
    if data.is_some() {
        let (drop_tx, drop_rx) = flume::unbounded();
        let data_array = data_to_arrow_array(data.clone(), metadata, drop_tx)
            .context("failed to convert output to arrow array")?;
        // integration testing doesn't use shared memory -> no drop tokens
        let _ = drop_rx;

        let data_type_json = serde_json::to_value(data_array.data_type())
            .context("failed to serialize data type as JSON")?;

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
        output.insert("data_type".into(), data_type_json);
    }
    Ok(output)
}

fn read_input_data(data: InputData) -> eyre::Result<arrow::array::ArrayData> {
    Ok(match data {
        InputData::JsonObject { data, data_type } => {
            // input is JSON data
            let array = json_value_to_list(data);
            let schema = match data_type {
                Some(ty) => data_type_to_schema(ty)?,
                None => arrow_json::reader::infer_json_schema_from_iterator(array.iter().map(Ok))?,
            };
            let schema = Arc::new(schema);
            read_json_value_as_arrow(&array, schema.clone()).with_context(|| {
                format!(
                    "failed to decode JSON value for data type {}",
                    schema
                        .fields()
                        .first()
                        .map(|f| f.data_type())
                        .unwrap_or(&DataType::Null)
                )
            })?
        }
        InputData::ArrowFile {
            path,
            batch_index,
            column,
        } => {
            let file = std::fs::File::open(&path)
                .with_context(|| format!("failed to open arrow file {}", path.display()))?;
            let mut reader = arrow::ipc::reader::FileReader::try_new(file, None)
                .context("failed to create arrow file reader")?;
            reader.set_index(batch_index).with_context(|| {
                format!(
                    "failed to seek to batch index {} in arrow file {}",
                    batch_index,
                    path.display()
                )
            })?;
            let batch = reader
                .next()
                .context("no batch at given index")?
                .context("failed to read batch from arrow file")?;
            match column {
                Some(name) => batch
                    .column_by_name(&name)
                    .with_context(|| {
                        format!(
                            "failed to find column '{}' in batch at index {} of arrow file {}",
                            name,
                            batch_index,
                            path.display()
                        )
                    })?
                    .to_data(),
                None => StructArray::from(batch).to_data(),
            }
        }
    })
}

fn json_value_to_list(value: serde_json::Value) -> Vec<serde_json::Value> {
    match value {
        serde_json::Value::Array(inner) => inner.into_iter().map(wrap_value_into_object).collect(),
        _ => {
            // wrap into object to allow bare values
            let object = wrap_value_into_object(value);
            vec![object]
        }
    }
}

fn data_type_to_schema(ty: serde_json::Value) -> eyre::Result<Schema> {
    let ty = serde_json::from_value::<DataType>(ty)
        .context("failed to deserialize `type` field of input data")?;
    Ok(Schema::new([Arc::new(Field::new("inner", ty, true))]))
}

fn wrap_value_into_object(value: serde_json::Value) -> serde_json::Value {
    let mut map = serde_json::Map::new();
    map.insert("inner".into(), value);

    serde_json::Value::Object(map)
}
