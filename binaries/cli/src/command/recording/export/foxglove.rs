//! `dora recording export --schema foxglove`: re-encode image outputs as
//! Foxglove's well-known `foxglove.RawImage` / `foxglove.CompressedImage`
//! schemas, JSON-encoded, so Foxglove can display them. The Arrow channels
//! the default export writes are valid MCAP but Foxglove does not render
//! Arrow (#3489, tier 2).
//!
//! An output is an image when the dataflow says so: `output_types` in the
//! descriptor stored in the recording header declares `std/media/v1/Image` or
//! `std/media/v1/CompressedImage`. An undeclared output whose messages carry
//! `width` / `height` / `encoding` metadata next to a flat `uint8` payload (the
//! camera convention in the examples) is treated as a raw image too.
use std::collections::HashMap;
use std::io::Cursor;

use arrow::array::{Array, ArrayRef, AsArray};
use arrow::datatypes::{DataType, UInt8Type};
use arrow::ipc::reader::StreamReader;
use base64::Engine as _;
use dora_message::metadata::{Metadata, Parameter};

/// A Foxglove schema this export can produce.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(super) enum Kind {
    RawImage,
    CompressedImage,
}

impl Kind {
    pub(super) fn schema_name(self) -> &'static str {
        match self {
            Kind::RawImage => "foxglove.RawImage",
            Kind::CompressedImage => "foxglove.CompressedImage",
        }
    }

    /// The JSON schema for the MCAP `Schema` record. Foxglove picks its
    /// renderer by the schema name; this describes the fields we write.
    pub(super) fn json_schema(self) -> &'static str {
        match self {
            Kind::RawImage => RAW_IMAGE_SCHEMA,
            Kind::CompressedImage => COMPRESSED_IMAGE_SCHEMA,
        }
    }
}

/// The `timestamp` field both schemas share.
macro_rules! time_schema {
    () => {
        r#"{"type":"object","properties":{"sec":{"type":"integer","minimum":0},"nsec":{"type":"integer","minimum":0,"maximum":999999999}}}"#
    };
}

const RAW_IMAGE_SCHEMA: &str = concat!(
    r#"{"title":"foxglove.RawImage","type":"object","properties":{"timestamp":"#,
    time_schema!(),
    r#","frame_id":{"type":"string"},"width":{"type":"integer","minimum":0},"height":{"type":"integer","minimum":0},"#,
    r#""encoding":{"type":"string"},"step":{"type":"integer","minimum":0},"data":{"type":"string","contentEncoding":"base64"}}}"#
);

const COMPRESSED_IMAGE_SCHEMA: &str = concat!(
    r#"{"title":"foxglove.CompressedImage","type":"object","properties":{"timestamp":"#,
    time_schema!(),
    r#","frame_id":{"type":"string"},"data":{"type":"string","contentEncoding":"base64"},"format":{"type":"string"}}}"#
);

/// What the mapper made of one recorded message.
pub(super) enum Recognition {
    /// Not an image output: export it as Arrow like the default schema does.
    NotRecognised,
    /// The JSON message for this Foxglove schema.
    Converted(Kind, Vec<u8>),
    /// An image output, but this message could not be converted.
    Failed(Kind, String),
}

enum Declared {
    Image,
    CompressedImage { format: Option<String> },
}

/// Which outputs are images, from the dataflow descriptor in the recording.
pub(super) struct Mapper {
    /// node id -> output id -> declared image type
    declared: HashMap<String, HashMap<String, Declared>>,
}

impl Mapper {
    /// Reads `nodes[].output_types` from the recorded descriptor. A descriptor
    /// that does not parse just declares nothing: export still works, using
    /// the metadata convention only.
    pub(super) fn from_descriptor_yaml(yaml: &str) -> Self {
        let mut declared: HashMap<String, HashMap<String, Declared>> = HashMap::new();
        let Ok(descriptor) = serde_yaml::from_str::<serde_yaml::Value>(yaml) else {
            return Self { declared };
        };
        let nodes = descriptor
            .get("nodes")
            .and_then(|nodes| nodes.as_sequence());
        for node in nodes.into_iter().flatten() {
            let Some(node_id) = node.get("id").and_then(|id| id.as_str()) else {
                continue;
            };
            let types = node.get("output_types").and_then(|t| t.as_mapping());
            for (output_id, urn) in types.into_iter().flatten() {
                let (Some(output_id), Some(urn)) = (output_id.as_str(), urn.as_str()) else {
                    continue;
                };
                let Some(urn) = dora_core::types::parse_urn(urn) else {
                    continue;
                };
                let image = match urn.base.as_str() {
                    "std/media/v1/Image" | "Image" => Declared::Image,
                    "std/media/v1/CompressedImage" | "CompressedImage" => {
                        Declared::CompressedImage {
                            format: urn.params.get("format").cloned(),
                        }
                    }
                    _ => continue,
                };
                declared
                    .entry(node_id.to_owned())
                    .or_default()
                    .insert(output_id.to_owned(), image);
            }
        }
        Self { declared }
    }

    pub(super) fn convert(
        &self,
        node_id: &str,
        output_id: &str,
        metadata: &Metadata,
        data: Option<&[u8]>,
    ) -> Recognition {
        let declared = self
            .declared
            .get(node_id)
            .and_then(|outputs| outputs.get(output_id));
        let (kind, json) = match declared {
            Some(Declared::Image) => (Kind::RawImage, image_struct(metadata, data)),
            Some(Declared::CompressedImage { format }) => (
                Kind::CompressedImage,
                compressed_image(metadata, data, format.as_deref()),
            ),
            None => match camera_frame(metadata, data) {
                Some(json) => (Kind::RawImage, json),
                None => return Recognition::NotRecognised,
            },
        };
        match json {
            Ok(json) => Recognition::Converted(kind, json),
            Err(reason) => Recognition::Failed(kind, reason),
        }
    }
}

/// A declared `std/media/v1/Image`: a one-row struct of
/// `width`, `height`, `encoding`, `data`.
fn image_struct(metadata: &Metadata, data: Option<&[u8]>) -> Result<Vec<u8>, String> {
    let array = decode(data)?;
    let image = array
        .as_struct_opt()
        .ok_or_else(|| format!("expected a struct, got {}", array.data_type()))?;
    if image.len() != 1 {
        return Err(format!(
            "expected one image per message, got {}",
            image.len()
        ));
    }
    let field = |name: &str| {
        image
            .column_by_name(name)
            .ok_or_else(|| format!("image struct has no `{name}` field"))
    };
    let width = unsigned(field("width")?.as_ref(), "width")?;
    let height = unsigned(field("height")?.as_ref(), "height")?;
    let encoding_column = field("encoding")?;
    let encoding = match encoding_column.data_type() {
        DataType::Utf8 => encoding_column.as_string::<i32>().value(0),
        DataType::LargeUtf8 => encoding_column.as_string::<i64>().value(0),
        other => return Err(format!("`encoding` must be a string, got {other}")),
    };
    let pixels = bytes(field("data")?.as_ref())
        .ok_or_else(|| "`data` must be binary or uint8".to_string())?;
    raw_image(metadata, width, height, encoding, pixels)
}

/// The camera convention: a flat `uint8` payload with `width`, `height` and
/// `encoding` in the metadata. `None` when the metadata does not describe an
/// image, so the output is exported as Arrow.
fn camera_frame(metadata: &Metadata, data: Option<&[u8]>) -> Option<Result<Vec<u8>, String>> {
    let params = &metadata.parameters;
    let (
        Some(Parameter::Integer(width)),
        Some(Parameter::Integer(height)),
        Some(Parameter::String(encoding)),
    ) = (
        params.get("width"),
        params.get("height"),
        params.get("encoding"),
    )
    else {
        return None;
    };
    Some((|| {
        let width = u64::try_from(*width).map_err(|_| format!("negative width {width}"))?;
        let height = u64::try_from(*height).map_err(|_| format!("negative height {height}"))?;
        let array = decode(data)?;
        let pixels = bytes(array.as_ref())
            .ok_or_else(|| format!("expected uint8 pixels, got {}", array.data_type()))?;
        raw_image(metadata, width, height, encoding, pixels)
    })())
}

fn raw_image(
    metadata: &Metadata,
    width: u64,
    height: u64,
    encoding: &str,
    pixels: &[u8],
) -> Result<Vec<u8>, String> {
    let rows = usize::try_from(height).map_err(|_| format!("height {height} is too large"))?;
    if rows == 0 || !pixels.len().is_multiple_of(rows) {
        return Err(format!(
            "{} bytes of pixel data are not {height} equal rows",
            pixels.len()
        ));
    }
    let step = pixels.len() / rows;
    if let Some(bytes_per_pixel) = bytes_per_pixel(encoding)
        && (step as u64) < width.saturating_mul(bytes_per_pixel)
    {
        return Err(format!(
            "rows of {step} bytes cannot hold {width} `{encoding}` pixels"
        ));
    }
    let json = serde_json::json!({
        "timestamp": timestamp(metadata),
        "frame_id": frame_id(metadata),
        "width": width,
        "height": height,
        "encoding": encoding,
        "step": step,
        "data": base64::engine::general_purpose::STANDARD.encode(pixels),
    });
    serde_json::to_vec(&json).map_err(|err| err.to_string())
}

/// Bytes per pixel of the packed encodings Foxglove lists for `RawImage`;
/// `None` for the subsampled YUV ones and anything unknown, which are not
/// checked.
fn bytes_per_pixel(encoding: &str) -> Option<u64> {
    match encoding {
        "mono8" | "8UC1" | "bayer_rggb8" | "bayer_bggr8" | "bayer_gbrg8" | "bayer_grbg8" => Some(1),
        "mono16" | "16UC1" => Some(2),
        "rgb8" | "bgr8" | "8UC3" => Some(3),
        "rgba8" | "bgra8" | "32FC1" => Some(4),
        _ => None,
    }
}

/// A declared `std/media/v1/CompressedImage`: one binary value. The format is
/// the URN's `format` parameter, else a `format` metadata parameter, else
/// detected from the file signature.
fn compressed_image(
    metadata: &Metadata,
    data: Option<&[u8]>,
    declared_format: Option<&str>,
) -> Result<Vec<u8>, String> {
    let array = decode(data)?;
    let encoded = bytes(array.as_ref())
        .ok_or_else(|| format!("expected binary image data, got {}", array.data_type()))?;
    let format = match (declared_format, metadata.parameters.get("format")) {
        (Some(format), _) => format,
        (None, Some(Parameter::String(format))) => format.as_str(),
        _ => detect_format(encoded)
            .ok_or_else(|| "unknown compressed image format; declare it as `std/media/v1/CompressedImage[format=jpeg]`".to_string())?,
    };
    let format = match format.to_ascii_lowercase().as_str() {
        "jpg" => "jpeg".to_owned(),
        other => other.to_owned(),
    };
    let json = serde_json::json!({
        "timestamp": timestamp(metadata),
        "frame_id": frame_id(metadata),
        "data": base64::engine::general_purpose::STANDARD.encode(encoded),
        "format": format,
    });
    serde_json::to_vec(&json).map_err(|err| err.to_string())
}

/// The formats Foxglove's `CompressedImage` accepts, by file signature.
fn detect_format(data: &[u8]) -> Option<&'static str> {
    if data.starts_with(&[0xFF, 0xD8, 0xFF]) {
        Some("jpeg")
    } else if data.starts_with(&[0x89, b'P', b'N', b'G', 0x0D, 0x0A, 0x1A, 0x0A]) {
        Some("png")
    } else if data.len() >= 12 && &data[..4] == b"RIFF" && &data[8..12] == b"WEBP" {
        Some("webp")
    } else if data.len() >= 12 && &data[4..12] == b"ftypavif" {
        Some("avif")
    } else {
        None
    }
}

/// The single array a dora output carries: column 0 of the one record batch
/// in its Arrow IPC payload.
fn decode(data: Option<&[u8]>) -> Result<ArrayRef, String> {
    let data = data.ok_or_else(|| "message has no payload".to_string())?;
    let mut reader = StreamReader::try_new(Cursor::new(data), None)
        .map_err(|err| format!("payload is not an Arrow IPC stream: {err}"))?;
    let batch = reader
        .next()
        .ok_or_else(|| "payload has no record batch".to_string())?
        .map_err(|err| format!("failed to read the payload: {err}"))?;
    if reader.next().is_some() {
        return Err("payload has more than one record batch".to_string());
    }
    if batch.num_columns() != 1 {
        return Err(format!("expected one column, got {}", batch.num_columns()));
    }
    Ok(batch.column(0).clone())
}

/// The bytes of a one-value binary array or of a flat `uint8` array.
fn bytes(array: &dyn Array) -> Option<&[u8]> {
    match array.data_type() {
        DataType::Binary if array.len() == 1 => Some(array.as_binary::<i32>().value(0)),
        DataType::LargeBinary if array.len() == 1 => Some(array.as_binary::<i64>().value(0)),
        DataType::UInt8 if array.null_count() == 0 => {
            Some(array.as_primitive::<UInt8Type>().values())
        }
        _ => None,
    }
}

fn unsigned(array: &dyn Array, name: &str) -> Result<u64, String> {
    use arrow::datatypes::{Int32Type, Int64Type, UInt32Type, UInt64Type};
    let value = match array.data_type() {
        DataType::UInt32 => u64::from(array.as_primitive::<UInt32Type>().value(0)),
        DataType::UInt64 => array.as_primitive::<UInt64Type>().value(0),
        DataType::Int32 => u64::try_from(array.as_primitive::<Int32Type>().value(0))
            .map_err(|_| format!("negative `{name}`"))?,
        DataType::Int64 => u64::try_from(array.as_primitive::<Int64Type>().value(0))
            .map_err(|_| format!("negative `{name}`"))?,
        other => return Err(format!("`{name}` must be an integer, got {other}")),
    };
    Ok(value)
}

/// The producer's HLC stamp, the same time the export uses as `publish_time`.
fn timestamp(metadata: &Metadata) -> serde_json::Value {
    let time = metadata.timestamp().get_time().to_duration();
    serde_json::json!({ "sec": time.as_secs(), "nsec": time.subsec_nanos() })
}

/// The `frame_id` metadata parameter, if the producer set one.
fn frame_id(metadata: &Metadata) -> &str {
    match metadata.parameters.get("frame_id") {
        Some(Parameter::String(frame_id)) => frame_id,
        _ => "",
    }
}

#[cfg(test)]
mod tests {
    use super::{Kind, detect_format};

    #[test]
    fn schemas_are_json_named_after_the_foxglove_type() {
        for kind in [Kind::RawImage, Kind::CompressedImage] {
            let schema: serde_json::Value =
                serde_json::from_str(kind.json_schema()).expect("schema is JSON");
            assert_eq!(schema["title"], kind.schema_name());
            assert_eq!(
                schema["properties"]["data"]["contentEncoding"], "base64",
                "{kind:?}"
            );
            assert!(schema["properties"]["timestamp"]["properties"]["nsec"].is_object());
        }
    }

    #[test]
    fn detects_the_formats_foxglove_accepts() {
        assert_eq!(detect_format(&[0xFF, 0xD8, 0xFF, 0xDB]), Some("jpeg"));
        assert_eq!(
            detect_format(&[
                0, 0, 0, 0x1C, b'f', b't', b'y', b'p', b'a', b'v', b'i', b'f'
            ]),
            Some("avif")
        );
        assert_eq!(detect_format(b"GIF89a"), None);
    }
}
