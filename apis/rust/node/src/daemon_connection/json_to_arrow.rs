use std::{
    io::{BufRead, Read},
    sync::Arc,
};

use arrow::array::{Array, ArrayData};
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

    if batch.num_columns() == 0 {
        eyre::bail!("JSON record batch has no columns");
    }
    Ok(batch.column(0).to_data())
}

// wrap data into JSON object to also allow bare JSON values
fn wrapped(data: impl BufRead) -> impl BufRead {
    "{ \"inner\":".as_bytes().chain(data).chain("}".as_bytes())
}

// wrap data into JSON object to also allow bare JSON values
fn wrapped_quoted(data: impl BufRead) -> impl BufRead {
    let quoted = "\"".as_bytes().chain(data).chain("\"".as_bytes());
    wrapped(quoted)
}

/// convert the given JSON object to the closed arrow representation
pub fn read_json_value_as_arrow(
    data: &[serde_json::Value],
    schema: Arc<arrow_schema::Schema>,
) -> eyre::Result<ArrayData> {
    let mut decoder = arrow_json::reader::ReaderBuilder::new(schema)
        .build_decoder()
        .context("failed to build JSON decoder")?;
    decoder
        .serialize(data)
        .context("failed to decode JSON to arrow array")?;
    let batch = decoder
        .flush()
        .context("failed to read record batch")?
        .context("no record batch in JSON")?;
    if batch.num_columns() == 0 {
        eyre::bail!("JSON record batch has no columns");
    }
    Ok(batch.column(0).to_data())
}

#[cfg(test)]
mod tests {
    use super::*;
    use arrow_schema::Schema;

    #[test]
    fn empty_field_schema_errors_instead_of_panicking() {
        // A caller-supplied schema with no fields yields a zero-column record
        // batch; `column(0)` would otherwise index out of bounds and panic.
        let schema = Arc::new(Schema::empty());
        let values = [serde_json::json!({})];
        let result = read_json_value_as_arrow(&values, schema);
        assert!(result.is_err());
    }
}
