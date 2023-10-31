use chrono::{DateTime, Utc};
use dora_node_api::{
    self,
    arrow::{
        array::{
            make_array, Array, ListArray, StringArray, TimestampMillisecondArray, UInt64Array,
        },
        buffer::{OffsetBuffer, ScalarBuffer},
        datatypes::{DataType, Field, Schema},
        ipc::writer::FileWriter,
        record_batch::RecordBatch,
    },
    DoraNode, Event, Metadata,
};
use dora_tracing::telemetry::deserialize_to_hashmap;
use eyre::{Context, ContextCompat};
use std::{collections::HashMap, fs::File, sync::Arc};

fn main() -> eyre::Result<()> {
    let (_node, mut events) = DoraNode::init_from_env()?;

    let mut writers = HashMap::new();
    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, data, metadata } => {
                match writers.get_mut(&id) {
                    None => {
                        let field_uhlc = Field::new("timestamp_uhlc", DataType::UInt64, false);
                        let field_utc_epoch = Field::new(
                            "timestamp_utc",
                            DataType::Timestamp(
                                dora_node_api::arrow::datatypes::TimeUnit::Millisecond,
                                None,
                            ),
                            false,
                        );
                        let field_trace_id = Field::new("trace_id", DataType::Utf8, true);
                        let field_span_id = Field::new("span_id", DataType::Utf8, true);
                        let field_values =
                            Arc::new(Field::new("item", data.data_type().clone(), true));
                        let field_data = Field::new(id.clone(), DataType::List(field_values), true);

                        let schema = Arc::new(Schema::new(vec![
                            field_trace_id,
                            field_span_id,
                            field_uhlc,
                            field_utc_epoch,
                            field_data,
                        ]));
                        let file = std::fs::File::create(format!("{id}.arrow")).unwrap();

                        let writer = FileWriter::try_new(file, &schema).unwrap();
                        let mut writer = writer;
                        write_event(&mut writer, data.into(), &metadata)
                            .context("could not write first record data")?;
                        writers.insert(id.clone(), writer);
                    }
                    Some(writer) => {
                        write_event(writer, data.into(), &metadata)
                            .context("could not write record data")?;
                    }
                };
            }
            Event::InputClosed { id } => match writers.remove(&id) {
                None => {}
                Some(mut writer) => writer.finish().context("Could not finish arrow file")?,
            },
            _ => {}
        }
    }

    let result: eyre::Result<Vec<_>> = writers
        .iter_mut()
        .map(|(_, writer)| -> eyre::Result<()> {
            writer
                .finish()
                .context("Could not finish writing arrow file")?;
            Ok(())
        })
        .collect();
    result.context("At least one of the input recorder file writer failed to finish")?;

    Ok(())
}

/// Write a row of data into the writer
fn write_event(
    writer: &mut FileWriter<File>,
    data: Arc<dyn Array>,
    metadata: &Metadata,
) -> eyre::Result<()> {
    let offsets = OffsetBuffer::new(ScalarBuffer::from(vec![0, data.len() as i32]));
    let field = Arc::new(Field::new("item", data.data_type().clone(), true));
    let list = ListArray::new(field, offsets, data.clone(), None);

    let timestamp = metadata.timestamp();
    let timestamp_uhlc = UInt64Array::from(vec![timestamp.get_time().0]);
    let timestamp_uhlc = make_array(timestamp_uhlc.into());
    let system_time = timestamp.get_time().to_system_time();

    let dt: DateTime<Utc> = system_time.into();
    let timestamp_utc = TimestampMillisecondArray::from(vec![dt.timestamp_millis()]);
    let timestamp_utc = make_array(timestamp_utc.into());

    let string_otel_context = metadata.parameters.open_telemetry_context.to_string();
    let otel_context = deserialize_to_hashmap(&string_otel_context);
    let traceparent = otel_context.get("traceparent");
    let trace_id = match traceparent {
        None => "",
        Some(trace) => trace.split('-').nth(1).context("Trace is malformatted")?,
    };
    let span_id = match traceparent {
        None => "",
        Some(trace) => trace.split('-').nth(2).context("Trace is malformatted")?,
    };
    let trace_id_array = StringArray::from(vec![trace_id]);
    let trace_id_array = make_array(trace_id_array.into());
    let span_id_array = StringArray::from(vec![span_id]);
    let span_id_array = make_array(span_id_array.into());

    let record = RecordBatch::try_new(
        writer.schema().clone(),
        vec![
            trace_id_array,
            span_id_array,
            timestamp_uhlc,
            timestamp_utc,
            make_array(list.into()),
        ],
    )
    .context("Could not create record batch with the given data")?;
    writer
        .write(&record)
        .context("Could not write recordbatch to file")?;

    Ok(())
}
