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

    Ok(batch.column(0).to_data())
}

// wrap data into JSON object to also allow bare JSON values
fn wrapped(data: impl BufRead) -> impl BufRead {
    "{ \"inner\":".as_bytes().chain(data).chain("}".as_bytes())
}

// wrap data into a JSON object, treating the input as a bare string value.
//
// The raw bytes are JSON-encoded (not merely surrounded by quotes) so that
// strings containing `"`, `\`, newlines, or other control characters still
// yield *valid* JSON. Previously the bytes were wrapped in literal quotes
// without escaping, so any such character produced malformed JSON and made
// schema inference fail — meaning the advertised "bare string" input path
// only worked for strings with no JSON-special characters.
fn wrapped_quoted(data: &[u8]) -> impl BufRead {
    // `from_utf8_lossy` keeps non-UTF-8 input producing valid JSON (via the
    // replacement character) instead of failing outright. `to_string` on a
    // string value cannot fail, but fall back defensively just in case.
    let s = String::from_utf8_lossy(data);
    let escaped = serde_json::to_string(s.as_ref()).unwrap_or_else(|_| "\"\"".to_string());
    wrapped(std::io::Cursor::new(escaped.into_bytes()))
}

#[cfg(test)]
mod tests {
    use super::read_json_bytes_as_arrow;
    use arrow::array::{Array, StringArray};

    fn parse_string(input: &str) -> String {
        let data = read_json_bytes_as_arrow(input.as_bytes())
            .unwrap_or_else(|e| panic!("failed to parse {input:?}: {e}"));
        let array = arrow::array::make_array(data);
        let strings = array
            .as_any()
            .downcast_ref::<StringArray>()
            .expect("expected a string array for bare-string input");
        assert_eq!(strings.len(), 1);
        strings.value(0).to_string()
    }

    #[test]
    fn bare_string_with_embedded_quotes() {
        // Regression: the string fallback used to wrap the raw bytes in literal
        // quotes without escaping, so any embedded `"` produced invalid JSON.
        assert_eq!(parse_string(r#"he said "hi""#), r#"he said "hi""#);
    }

    #[test]
    fn bare_string_with_backslash_and_newline() {
        assert_eq!(parse_string("a\\b\nc"), "a\\b\nc");
    }

    #[test]
    fn plain_string_still_works() {
        assert_eq!(parse_string("hello"), "hello");
    }
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
