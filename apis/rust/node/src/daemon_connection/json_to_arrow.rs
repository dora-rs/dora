use std::{
    io::{BufRead, Read},
    sync::Arc,
};

use arrow::array::{Array, ArrayData, RecordBatch};
use eyre::{Context, ContextCompat};

/// Extract the first column of `batch` as [`ArrayData`].
///
/// `RecordBatch::column(0)` panics on a batch with no columns; both JSON
/// readers can be handed (or infer) a zero-field schema, so return an error
/// instead of panicking.
fn first_column_data(batch: &RecordBatch) -> eyre::Result<ArrayData> {
    Ok(batch
        .columns()
        .first()
        .context("JSON record batch has no columns")?
        .to_data())
}

pub fn read_json_bytes_as_arrow(data: &[u8]) -> eyre::Result<ArrayData> {
    match arrow_json::reader::infer_json_schema(wrapped(data), None) {
        Ok((schema, _)) => read_from_json_with_schema(wrapped(data), schema),
        Err(_) => {
            // Try again by treating the input as a plain string. The bytes may
            // contain characters that are not valid on their own inside a JSON
            // string literal (`"`, `\`, raw control characters), so JSON-escape
            // them first instead of wrapping the raw bytes in quotes — otherwise
            // any such input produces invalid JSON and is rejected.
            let quoted = json_quote(data);
            match arrow_json::reader::infer_json_schema(wrapped(quoted.as_bytes()), None) {
                Ok((schema, _)) => read_from_json_with_schema(wrapped(quoted.as_bytes()), schema),
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

    first_column_data(&batch)
}

// wrap data into JSON object to also allow bare JSON values
fn wrapped(data: impl BufRead) -> impl BufRead {
    "{ \"inner\":".as_bytes().chain(data).chain("}".as_bytes())
}

/// JSON-encode the given bytes as a quoted string literal, escaping any
/// characters that are not valid inside a JSON string (`"`, `\`, control
/// characters). Non-UTF-8 bytes are replaced lossily.
fn json_quote(data: &[u8]) -> String {
    // Serializing a `String` into JSON never fails.
    serde_json::Value::String(String::from_utf8_lossy(data).into_owned()).to_string()
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
    first_column_data(&batch)
}

#[cfg(test)]
mod tests {
    use super::*;
    use arrow::array::{StringArray, make_array};
    use arrow_schema::Schema;

    fn as_single_string(data: ArrayData) -> String {
        let array = make_array(data);
        let strings = array
            .as_any()
            .downcast_ref::<StringArray>()
            .expect("expected a string array");
        assert_eq!(strings.len(), 1);
        strings.value(0).to_owned()
    }

    #[test]
    fn empty_field_schema_errors_instead_of_panicking() {
        // A caller-supplied schema with no fields yields a zero-column record
        // batch; `column(0)` would otherwise index out of bounds and panic.
        let schema = Arc::new(Schema::empty());
        let values = [serde_json::json!({})];
        let result = read_json_value_as_arrow(&values, schema);
        assert!(result.is_err());
    }

    #[test]
    fn single_field_schema_reads_first_column() {
        // The zero-column guard must not reject a normal single-field schema.
        let schema = Arc::new(Schema::new(vec![arrow_schema::Field::new(
            "value",
            arrow_schema::DataType::Int64,
            false,
        )]));
        let values = [serde_json::json!({ "value": 7 })];
        let array = read_json_value_as_arrow(&values, schema).expect("valid single-field decode");
        assert_eq!(array.len(), 1);
    }

    #[test]
    fn plain_text_with_double_quotes_round_trips() {
        // Previously this produced invalid JSON (`{ "inner":"say "hi""}`) and
        // was rejected instead of being treated as a plain string.
        let data = read_json_bytes_as_arrow(br#"say "hi""#).unwrap();
        assert_eq!(as_single_string(data), r#"say "hi""#);
    }

    #[test]
    fn plain_text_with_backslash_round_trips() {
        let data = read_json_bytes_as_arrow(br"C:\Users\node").unwrap();
        assert_eq!(as_single_string(data), r"C:\Users\node");
    }

    #[test]
    fn plain_text_with_control_characters_round_trips() {
        let data = read_json_bytes_as_arrow(b"line1\nline2\ttab").unwrap();
        assert_eq!(as_single_string(data), "line1\nline2\ttab");
    }

    #[test]
    fn bare_json_values_still_parse_natively() {
        // A bare JSON number is parsed as a number, not wrapped as a string.
        let data = read_json_bytes_as_arrow(b"42").unwrap();
        let array = make_array(data);
        assert_eq!(array.len(), 1);
        assert!(
            array.as_any().downcast_ref::<StringArray>().is_none(),
            "bare JSON number should not be parsed as a string"
        );
    }
}
