use chrono::{DateTime, Utc};
use dora_node_api::{
    self,
    arrow::{
        array::{make_array, Array, Int64Array, ListArray, StringArray},
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

// Remove once arrow-rs 48.0 is published with access to writer schema.
// See: https://github.com/apache/arrow-rs/pull/4940
struct WriterContext {
    writer: FileWriter<File>,
    schema: Arc<Schema>,
}

fn main() -> eyre::Result<()> {
    let (_node, mut events) = DoraNode::init_from_env()?;

    let mut writers = HashMap::new();
    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, data, metadata } => {
                match writers.get_mut(&id) {
                    None => {
                        let field_timestamp = Field::new("timestamp", DataType::Int64, true);
                        let field_otel_context = Field::new("otel_context", DataType::Utf8, true);
                        let field_values =
                            Arc::new(Field::new("item", data.data_type().clone(), true));
                        let field_data = Field::new(id.clone(), DataType::List(field_values), true);

                        let schema = Arc::new(Schema::new(vec![
                            field_otel_context,
                            field_timestamp,
                            field_data,
                        ]));
                        let file = std::fs::File::create(format!("{id}.arrow")).unwrap();

                        let writer = FileWriter::try_new(file, &schema).unwrap();
                        let mut writer_context = WriterContext { writer, schema };
                        write_event(&mut writer_context, data.into(), &metadata)
                            .context("could not write first record data")?;
                        writers.insert(id.clone(), writer_context);
                    }
                    Some(writer_context) => {
                        write_event(writer_context, data.into(), &metadata)
                            .context("could not write first record data")?;
                    }
                };
            }
            Event::InputClosed { id } => match writers.remove(&id) {
                None => {}
                Some(mut writer) => writer
                    .writer
                    .finish()
                    .context("Could not finish arrow file")?,
            },
            _ => {}
        }
    }

    let result: eyre::Result<Vec<_>> = writers
        .iter_mut()
        .map(|(_, wc)| -> eyre::Result<()> {
            wc.writer
                .finish()
                .context("Could not finish writing arrow file")?;
            Ok(())
        })
        .collect();
    result.context("One of the input recorder file writer failed to finish")?;

    Ok(())
}

fn write_event(
    writer_context: &mut WriterContext,
    data: Arc<dyn Array>,
    metadata: &Metadata,
) -> eyre::Result<()> {
    let offsets = OffsetBuffer::new(ScalarBuffer::from(vec![0, data.len() as i32]));
    let field = Arc::new(Field::new("item", data.data_type().clone(), true));
    let list = ListArray::new(field.clone(), offsets, data.clone(), None);

    let timestamp = metadata.timestamp();
    let timestamp = timestamp.get_time().to_system_time();

    let dt: DateTime<Utc> = timestamp.into();
    let timestamp_array = Int64Array::from(vec![dt.timestamp_millis()]);
    let timestamp_array = make_array(timestamp_array.into());

    let string_otel_context = metadata.parameters.open_telemetry_context.to_string();
    let otel_context = deserialize_to_hashmap(&string_otel_context);
    let traceparent = otel_context.get("traceparent").clone();
    let trace_id = match traceparent {
        None => "",
        Some(trace) => trace.split("-").nth(1).context("Trace is malformatted")?,
    };
    let otel_context_array = StringArray::from(vec![trace_id]);
    let otel_context_array = make_array(otel_context_array.into());

    let record = RecordBatch::try_new(
        writer_context.schema.clone(),
        vec![otel_context_array, timestamp_array, make_array(list.into())],
    )
    .context("Could not create record batch with the given data")?;
    writer_context
        .writer
        .write(&record)
        .context("Could not write recordbatch to file")?;

    Ok(())
}
