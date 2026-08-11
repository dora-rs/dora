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

    Ok(batch.column(0).to_data())
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
    Ok(batch.column(0).to_data())
}

#[cfg(test)]
mod tests {
    use super::*;
    use arrow::array::{StringArray, make_array};

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
