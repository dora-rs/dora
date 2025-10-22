use std::{
    io::{BufRead, Read},
    sync::Arc,
};

use arrow::array::{Array, ArrayData, StructArray};
use arrow_integration_test::ArrowJson;
use eyre::{Context, ContextCompat};

pub fn read_json_bytes_as_arrow(data: &[u8]) -> eyre::Result<ArrayData> {
    match arrow_json::reader::infer_json_schema(wrapped(data), None) {
        Ok((schema, _)) => read_from_json_with_schema(wrapped(data), schema),
        Err(_) => {
            // try again with quoting the input to treat it as a string
            match arrow_json::reader::infer_json_schema(wrapped_quoted(data), None) {
                Ok((schema, _)) => read_from_json_with_schema(wrapped_quoted(data), schema),
                Err(err) => eyre::bail!("failed to infer JSON schema: {err}"),
            }
        }
    }
}

fn read_from_json_with_schema(
    data: impl BufRead,
    schema: arrow_schema::Schema,
) -> eyre::Result<ArrayData> {
    let mut reader = arrow_json::reader::ReaderBuilder::new(Arc::new(schema))
        .build(data)
        .context("failed to build JSON reader")?;
    let batch = reader
        .next()
        .context("no record batch in JSON")?
        .context("failed to read record batch")?;

    Ok(batch.column(0).to_data())
}

// wrap data into JSON object to also allow bare JSON values
fn wrapped(data: impl BufRead) -> impl BufRead {
    "{ \"inner\":".as_bytes().chain(data).chain("}".as_bytes())
}

// wrap data into JSON object to also allow bare JSON values
fn wrapped_quoted(data: impl BufRead) -> impl BufRead {
    let quoted = [b'"'].chain(data).chain([b'"'].as_slice());
    wrapped(quoted)
}

pub fn convert_arrow_json_data(
    value: serde_json::Value,
    unwrap_first_column: bool,
) -> eyre::Result<ArrayData> {
    let data: ArrowJson =
        serde_json::from_value(value).context("failed to deserialize ArrowJson format")?;
    let batches = data
        .get_record_batches()
        .context("failed to get record batches of given ArrowJson data")?;

    if batches.len() > 1 {
        eyre::bail!("more than one RecordBatch given in arrow_data field (Dora only supports one)");
    }

    let batch = batches
        .into_iter()
        .next()
        .context("no record batch given")?;
    if unwrap_first_column {
        if batch.columns().len() > 1 {
            eyre::bail!(
                "given data has more than one column, use the `arrow_test` data format instead"
            );
        }
        Ok(batch.column(0).to_data())
    } else {
        Ok(StructArray::from(batch).to_data())
    }
}

/// convert the given JSON object to the closed arrow representation
pub fn read_json_value_as_arrow(data: &serde_json::Value) -> eyre::Result<ArrayData> {
    let schema = arrow_json::reader::infer_json_schema_from_iterator(std::iter::once(data).map(Ok))
        .context("failed to infer JSON schema")?;
    let mut decoder = arrow_json::reader::ReaderBuilder::new(Arc::new(schema))
        .build_decoder()
        .context("failed to build JSON decoder")?;
    decoder
        .serialize(&[data])
        .context("failed to decode JSON to arrow array")?;
    let batch = decoder
        .flush()
        .context("failed to read record batch")?
        .context("no record batch in JSON")?;
    Ok(batch.column(0).to_data())
}
