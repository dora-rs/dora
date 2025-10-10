use std::{
    io::{BufRead, Read},
    sync::Arc,
};

use arrow::array::{Array, ArrayData};
use eyre::{Context, ContextCompat};

pub fn read_json_as_arrow(data: &[u8]) -> eyre::Result<ArrayData> {
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
