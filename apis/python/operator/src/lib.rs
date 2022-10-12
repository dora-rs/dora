use std::borrow::Cow;

use dora_node_api::Metadata;
use eyre::{Context, Result};
use pyo3::{prelude::*, types::PyDict};

pub fn pydict_to_metadata<'a>(dict: Option<&'a PyDict>) -> Result<Metadata<'a>> {
    let mut default_metadata = Metadata::default();
    if let Some(metadata) = dict {
        for (key, value) in metadata.iter() {
            match key.extract::<&str>().context("Parsing metadata keys")? {
                "metadata_version" => {
                    default_metadata.metadata_version =
                        value.extract().context("parsing metadata version failed")?;
                }
                "watermark" => {
                    default_metadata.watermark =
                        value.extract().context("parsing watermark failed")?;
                }
                "deadline" => {
                    default_metadata.deadline =
                        value.extract().context("parsing deadline failed")?;
                }
                "open_telemetry_context" => {
                    let otel_context: &str = value
                        .extract()
                        .context("parsing open telemetry context failed")?;
                    default_metadata.open_telemetry_context = Cow::Borrowed(otel_context);
                }
                _ => (),
            }
        }
    }
    Ok(default_metadata)
}

pub fn metadata_to_pydict<'a>(metadata: &'a Metadata, py: Python<'a>) -> &'a PyDict {
    let dict = PyDict::new(py);
    dict.set_item("open_telemetry_context", &metadata.open_telemetry_context)
        .wrap_err("could not make metadata a python dictionary item")
        .unwrap();
    dict
}
