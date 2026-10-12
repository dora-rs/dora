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
    /// `std/media/v1/Image`, with its `encoding` parameter if given.
    Image {
        encoding: Option<String>,
    },
    CompressedImage {
        format: Option<String>,
    },
}

/// Which outputs are images, from the dataflow descriptor in the recording.
pub(super) struct Mapper {
    /// node id -> output id -> declared image type, keyed the way outputs are
    /// recorded: a single `operator:` node's outputs by their bare id, the
    /// outputs of an `operators:` list as `<operator id>/<output id>`.
    declared: HashMap<String, HashMap<String, Declared>>,
}

impl Mapper {
    /// Reads the `output_types` of each node and of its operators from the
    /// recorded descriptor (the user's YAML). A descriptor that does not parse
    /// just declares nothing: export still works, using the camera metadata
    /// convention only.
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
            let outputs = declared.entry(node_id.to_owned()).or_default();
            add_declared(outputs, node, None);
            // `dora record` and `--proxy` both record a single operator's
            // outputs without the operator id (`record.rs`
            // `discover_descriptor_outputs`, `public_topic_output_id`).
            if let Some(operator) = node.get("operator") {
                add_declared(outputs, operator, None);
            }
            let operators = node.get("operators").and_then(|ops| ops.as_sequence());
            for operator in operators.into_iter().flatten() {
                if let Some(operator_id) = operator.get("id").and_then(|id| id.as_str()) {
                    add_declared(outputs, operator, Some(operator_id));
                }
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
        match declared {
            Some(Declared::Image { encoding }) => {
                // Declared images come as the `Image` struct, or like the
                // README's camera as a flat array with camera metadata, or as
                // plain bytes when the type says `[encoding=jpeg]`.
                let result = match decode(data) {
                    Ok(array) if array.as_struct_opt().is_some() => image_struct(metadata, &array),
                    _ => match camera_frame(metadata, data) {
                        Some(result) => result,
                        None => match encoding.as_deref().and_then(compressed_format) {
                            Some(format) => compressed_bytes(metadata, data, format),
                            None => Err(
                                "expected an `Image` struct, or width/height/encoding metadata"
                                    .to_string(),
                            ),
                        },
                    },
                };
                match result {
                    Ok((kind, json)) => Recognition::Converted(kind, json),
                    Err(reason) => Recognition::Failed(Kind::RawImage, reason),
                }
            }
            Some(Declared::CompressedImage { format }) => {
                match compressed_image(metadata, data, format.as_deref()) {
                    Ok(json) => Recognition::Converted(Kind::CompressedImage, json),
                    Err(reason) => Recognition::Failed(Kind::CompressedImage, reason),
                }
            }
            // Not declared: only an image if it converts cleanly. Anything
            // else (no camera metadata, an encoding Foxglove can't show, a
            // payload that doesn't match) stays Arrow instead of being dropped.
            None => match camera_frame(metadata, data) {
                Some(Ok((kind, json))) => Recognition::Converted(kind, json),
                _ => Recognition::NotRecognised,
            },
        }
    }
}

/// Records the image types in `entry`'s `output_types`, keyed by output id,
/// prefixed with `operator_id/` for an operator.
fn add_declared(
    outputs: &mut HashMap<String, Declared>,
    entry: &serde_yaml::Value,
    operator_id: Option<&str>,
) {
    let types = entry.get("output_types").and_then(|t| t.as_mapping());
    for (output_id, urn) in types.into_iter().flatten() {
        let (Some(output_id), Some(urn)) = (output_id.as_str(), urn.as_str()) else {
            continue;
        };
        let Some(urn) = dora_core::types::parse_urn(urn) else {
            continue;
        };
        // Full URNs only: `dora validate` does not resolve short names in
        // `output_types` either.
        let image = match urn.base.as_str() {
            "std/media/v1/Image" => Declared::Image {
                encoding: urn.params.get("encoding").cloned(),
            },
            "std/media/v1/CompressedImage" => Declared::CompressedImage {
                format: urn.params.get("format").cloned(),
            },
            _ => continue,
        };
        let key = match operator_id {
            Some(operator_id) => format!("{operator_id}/{output_id}"),
            None => output_id.to_owned(),
        };
        outputs.insert(key, image);
    }
}

/// A declared `std/media/v1/Image` struct: one row of `width`, `height`,
/// `encoding`, `data`. A compressed `encoding` (jpeg, png, ...) makes it a
/// `CompressedImage`.
fn image_struct(metadata: &Metadata, array: &ArrayRef) -> Result<(Kind, Vec<u8>), String> {
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
        .ok_or_else(|| "`data` must be binary or a flat numeric array".to_string())?;
    if let Some(format) = compressed_format(encoding) {
        return Ok((
            Kind::CompressedImage,
            compressed_json(metadata, pixels, format)?,
        ));
    }
    Ok((
        Kind::RawImage,
        raw_image(metadata, width, height, encoding, pixels)?,
    ))
}

/// Plain compressed-image bytes (binary or flat `uint8`) in a known format.
fn compressed_bytes(
    metadata: &Metadata,
    data: Option<&[u8]>,
    format: &str,
) -> Result<(Kind, Vec<u8>), String> {
    let array = decode(data)?;
    let encoded = bytes(array.as_ref())
        .ok_or_else(|| format!("expected binary image data, got {}", array.data_type()))?;
    Ok((
        Kind::CompressedImage,
        compressed_json(metadata, encoded, format)?,
    ))
}

/// The camera convention (dora-hub's `opencv-video-capture`,
/// `dora-pyrealsense`, the examples): a flat array with `width`, `height` and
/// `encoding` metadata. JPEG/PNG/WebP/AVIF become `CompressedImage`, the raw
/// encodings Foxglove lists become `RawImage`. `None` when the metadata does
/// not describe an image.
fn camera_frame(
    metadata: &Metadata,
    data: Option<&[u8]>,
) -> Option<Result<(Kind, Vec<u8>), String>> {
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
        let array = decode(data)?;
        let pixels = bytes(array.as_ref())
            .ok_or_else(|| format!("expected a flat image array, got {}", array.data_type()))?;
        if let Some(format) = compressed_format(encoding) {
            return Ok((
                Kind::CompressedImage,
                compressed_json(metadata, pixels, format)?,
            ));
        }
        let width = u64::try_from(*width).map_err(|_| format!("negative width {width}"))?;
        let height = u64::try_from(*height).map_err(|_| format!("negative height {height}"))?;
        Ok((
            Kind::RawImage,
            raw_image(metadata, width, height, encoding, pixels)?,
        ))
    })())
}

fn raw_image(
    metadata: &Metadata,
    width: u64,
    height: u64,
    encoding: &str,
    pixels: &[u8],
) -> Result<Vec<u8>, String> {
    let bytes_per_pixel = raw_bytes_per_pixel(encoding)
        .ok_or_else(|| format!("`{encoding}` is not an encoding Foxglove can display"))?;
    // NV12 is a full-height Y plane followed by a half-height UV plane, so
    // its data holds 1.5 x `height` rows of `step` bytes.
    let rows = if encoding == "nv12" {
        height.saturating_add(height / 2)
    } else {
        height
    };
    let rows = usize::try_from(rows).map_err(|_| format!("height {height} is too large"))?;
    if rows == 0 || !pixels.len().is_multiple_of(rows) {
        return Err(format!(
            "{} bytes of `{encoding}` data are not whole rows of a {width}x{height} image",
            pixels.len()
        ));
    }
    let step = pixels.len() / rows;
    if (step as u64) < width.saturating_mul(bytes_per_pixel) {
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

/// Bytes per pixel (of the first plane) for the raw encodings Foxglove's
/// `RawImage` lists; `None` for anything it can't display.
fn raw_bytes_per_pixel(encoding: &str) -> Option<u64> {
    match encoding {
        "mono8" | "8UC1" | "nv12" | "bayer_rggb8" | "bayer_bggr8" | "bayer_gbrg8"
        | "bayer_grbg8" => Some(1),
        "mono16" | "16UC1" | "yuv422" | "uyvy" | "yuv422_yuy2" | "yuyv" => Some(2),
        "rgb8" | "bgr8" | "8UC3" => Some(3),
        "rgba8" | "bgra8" | "32FC1" => Some(4),
        _ => None,
    }
}

/// The `foxglove.CompressedImage` format for an encoding name, if it is one.
fn compressed_format(encoding: &str) -> Option<&'static str> {
    match encoding.to_ascii_lowercase().as_str() {
        "jpeg" | "jpg" | "jpe" => Some("jpeg"),
        "png" => Some("png"),
        "webp" => Some("webp"),
        "avif" => Some("avif"),
        _ => None,
    }
}

/// A declared `std/media/v1/CompressedImage`: one binary value. The format is
/// the URN's `format` parameter, else a `format` or `encoding` metadata
/// parameter, else detected from the file signature.
fn compressed_image(
    metadata: &Metadata,
    data: Option<&[u8]>,
    declared_format: Option<&str>,
) -> Result<Vec<u8>, String> {
    let array = decode(data)?;
    let encoded = bytes(array.as_ref())
        .ok_or_else(|| format!("expected binary image data, got {}", array.data_type()))?;
    let named = declared_format.or_else(|| {
        ["format", "encoding"]
            .iter()
            .find_map(|key| match metadata.parameters.get(*key) {
                Some(Parameter::String(value)) => Some(value.as_str()),
                _ => None,
            })
    });
    let format = match named {
        Some(name) => compressed_format(name)
            .ok_or_else(|| format!("`{name}` is not a format Foxglove can display"))?,
        None => detect_format(encoded).ok_or_else(|| {
            "unknown compressed image format; declare it as `std/media/v1/CompressedImage[format=jpeg]`"
                .to_string()
        })?,
    };
    compressed_json(metadata, encoded, format)
}

fn compressed_json(metadata: &Metadata, encoded: &[u8], format: &str) -> Result<Vec<u8>, String> {
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

/// The bytes of a one-value binary array, or the value buffer of a flat
/// array without nulls: `uint8` pixels, `uint16` / `int16` (e.g. `mono16`
/// depth) or `float32` (`32FC1`). Arrow buffers are little-endian on every
/// platform dora ships for, matching what Foxglove expects.
fn bytes(array: &dyn Array) -> Option<&[u8]> {
    use arrow::datatypes::{Float32Type, Int16Type, UInt16Type};
    if array.null_count() != 0 {
        return None;
    }
    match array.data_type() {
        DataType::Binary if array.len() == 1 => Some(array.as_binary::<i32>().value(0)),
        DataType::LargeBinary if array.len() == 1 => Some(array.as_binary::<i64>().value(0)),
        DataType::UInt8 => Some(
            array
                .as_primitive::<UInt8Type>()
                .values()
                .inner()
                .as_slice(),
        ),
        DataType::UInt16 => Some(
            array
                .as_primitive::<UInt16Type>()
                .values()
                .inner()
                .as_slice(),
        ),
        DataType::Int16 => Some(
            array
                .as_primitive::<Int16Type>()
                .values()
                .inner()
                .as_slice(),
        ),
        DataType::Float32 => Some(
            array
                .as_primitive::<Float32Type>()
                .values()
                .inner()
                .as_slice(),
        ),
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
