use chrono::{DateTime, Utc};
use dora_node_api::{
    self,
    arrow::{
        array::{
            make_array, Array, ListArray, StringArray, TimestampMillisecondArray, UInt64Array,
        },
        buffer::{OffsetBuffer, ScalarBuffer},
        datatypes::{DataType, Field, Schema},
        record_batch::RecordBatch,
    },
    DoraNode, Event, Metadata,
};
use dora_tracing::telemetry::deserialize_to_hashmap;
use eyre::{Context, ContextCompat};
use parquet::{arrow::AsyncArrowWriter, basic::BrotliLevel, file::properties::WriterProperties};
use std::{collections::HashMap, path::PathBuf, sync::Arc};
use tokio::sync::mpsc;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    let (node, mut events) = DoraNode::init_from_env()?;
    let dataflow_id = node.dataflow_id();
    let mut writers = HashMap::new();

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, data, metadata } => {
                match writers.get(&id) {
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
                        let dataflow_dir = PathBuf::from("out").join(dataflow_id.to_string());
                        if !dataflow_dir.exists() {
                            std::fs::create_dir_all(&dataflow_dir)
                                .context("could not create dataflow_dir")?;
                        }
                        let file =
                            tokio::fs::File::create(dataflow_dir.join(format!("{id}.parquet")))
                                .await
                                .context("Couldn't create write file")?;
                        let mut writer = AsyncArrowWriter::try_new(
                            file,
                            schema.clone(),
                            0,
                            Some(
                                WriterProperties::builder()
                                    .set_compression(parquet::basic::Compression::BROTLI(
                                        BrotliLevel::default(),
                                    ))
                                    .build(),
                            ),
                        )
                        .context("Could not create parquet writer")?;
                        let (tx, mut rx) = mpsc::channel(10);

                        // Per Input thread
                        let join_handle = tokio::spawn(async move {
                            while let Some((data, metadata)) = rx.recv().await {
                                if let Err(e) =
                                    write_event(&mut writer, data, &metadata, schema.clone()).await
                                {
                                    println!("Error writing event data into parquet file: {:?}", e)
                                };
                            }
                            writer.close().await
                        });
                        writers.insert(id, (tx, join_handle));
                    }
                    Some((tx, _)) => {
                        tx.send((data.into(), metadata))
                            .await
                            .context("Could not send event data into writer loop")?;
                    }
                };
            }
            Event::InputClosed { id } => match writers.remove(&id) {
                None => {}
                Some(tx) => drop(tx),
            },
            _ => {}
        }
    }

    for (id, (tx, join_handle)) in writers {
        drop(tx);
        join_handle
            .await
            .context("Writer thread failed")?
            .context(format!(
                "Could not close the Parquet writer for {id} parquet writer"
            ))?;
    }

    Ok(())
}

/// Write a row of data into the writer
async fn write_event(
    writer: &mut AsyncArrowWriter<tokio::fs::File>,
    data: Arc<dyn Array>,
    metadata: &Metadata,
    schema: Arc<Schema>,
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
        schema,
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
        .await
        .context("Could not write recordbatch to file")?;

    Ok(())
}
