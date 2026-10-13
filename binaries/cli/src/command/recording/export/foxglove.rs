//! `dora recording export --schema foxglove`: re-encode image outputs as
//! Foxglove's well-known `foxglove.RawImage` / `foxglove.CompressedImage`
//! schemas, JSON-encoded, so Foxglove can display them. The Arrow channels
//! the default export writes are valid MCAP but Foxglove does not render
//! Arrow (#3489, tier 2).
//!
//! An output is an image when the dataflow says so: `output_types` in the
//! descriptor stored in the recording header declares `std/media/v1/Image` or
//! `std/media/v1/CompressedImage`. An undeclared output is an image when its
//! messages follow the camera convention of the dora-hub nodes: a flat array
//! with `width` / `height` / `encoding` metadata, in an encoding Foxglove can
//! display. Undeclared compressed frames must also be bytes in the format the
//! metadata names, because nodes often forward a camera's metadata onto
//! outputs that are not images.
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
    /// Not an image: export it as Arrow like the default schema does. For an
    /// undeclared message that looked like a camera frame but didn't convert,
    /// the reason, for the report when its topic is already an image channel.
    NotRecognised(Option<String>),
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
    /// Any other declared type: not an image, never guessed at.
    Other,
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
            let Some(node_id) = node.get("id").and_then(yaml_id) else {
                continue;
            };
            let outputs = declared.entry(node_id).or_default();
            add_declared(outputs, node, None);
            // `dora record` and `--proxy` both record a single operator's
            // outputs without the operator id (`record.rs`
            // `discover_descriptor_outputs`, `public_topic_output_id`).
            if let Some(operator) = node.get("operator") {
                add_declared(outputs, operator, None);
            }
            let operators = node.get("operators").and_then(|ops| ops.as_sequence());
            for operator in operators.into_iter().flatten() {
                if let Some(operator_id) = operator.get("id").and_then(yaml_id) {
                    add_declared(outputs, operator, Some(&operator_id));
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
                // Declared images come as the `Image` struct, or as a flat
                // array: compressed bytes, or pixels with width/height
                // metadata. An encoding on the type (`Image[encoding=jpeg]`)
                // wins over one the node forwarded in its metadata.
                let encoding = encoding.as_deref();
                let result = decode(data).and_then(|array| {
                    if array.as_struct_opt().is_some() {
                        return image_struct(metadata, &array, encoding);
                    }
                    frame(metadata, encoding, &array, false)
                });
                let expected = match encoding
                    .or_else(|| metadata_str(metadata, "encoding"))
                    .and_then(compressed_format)
                {
                    Some(_) => Kind::CompressedImage,
                    None => Kind::RawImage,
                };
                match result {
                    Ok((kind, json)) => Recognition::Converted(kind, json),
                    Err(reason) => Recognition::Failed(expected, reason),
                }
            }
            Some(Declared::CompressedImage { format }) => {
                match decode(data)
                    .and_then(|array| compressed_image(metadata, &array, format.as_deref()))
                {
                    Ok(json) => Recognition::Converted(Kind::CompressedImage, json),
                    Err(reason) => Recognition::Failed(Kind::CompressedImage, reason),
                }
            }
            Some(Declared::Other) => Recognition::NotRecognised(None),
            // Not declared: only an image if it converts cleanly. Anything
            // else (no `encoding` metadata, an encoding Foxglove can't show,
            // a payload that doesn't match) stays Arrow instead of being
            // dropped. The metadata is checked first, so other outputs aren't
            // decoded.
            None => {
                if metadata_str(metadata, "encoding").is_none() {
                    return Recognition::NotRecognised(None);
                }
                match decode(data).and_then(|array| frame(metadata, None, &array, true)) {
                    Ok((kind, json)) => Recognition::Converted(kind, json),
                    Err(reason) => Recognition::NotRecognised(Some(reason)),
                }
            }
        }
    }
}

/// A node, operator or output id: dora reads a YAML number (`1:`) as the id
/// `1`, so it counts here too.
fn yaml_id(value: &serde_yaml::Value) -> Option<String> {
    match value {
        serde_yaml::Value::String(id) => Some(id.clone()),
        serde_yaml::Value::Number(id) => Some(id.to_string()),
        _ => None,
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
        let (Some(output_id), Some(urn)) = (yaml_id(output_id), urn.as_str()) else {
            continue;
        };
        let key = match operator_id {
            Some(operator_id) => format!("{operator_id}/{output_id}"),
            None => output_id.to_owned(),
        };
        // Full URNs only: `dora validate` does not resolve short names in
        // `output_types` either. Anything else, a malformed type included,
        // still declares the output as not an image.
        let image = match dora_core::types::parse_urn(urn) {
            Some(urn) if urn.base == "std/media/v1/Image" => Declared::Image {
                encoding: urn.params.get("encoding").cloned(),
            },
            Some(urn) if urn.base == "std/media/v1/CompressedImage" => Declared::CompressedImage {
                format: urn.params.get("format").cloned(),
            },
            _ => Declared::Other,
        };
        outputs.insert(key, image);
    }
}

/// A declared `std/media/v1/Image` struct: one row of `width`, `height`,
/// `encoding`, `data`. A compressed `encoding` (jpeg, png, ...) makes it a
/// `CompressedImage`.
fn image_struct(
    metadata: &Metadata,
    array: &ArrayRef,
    declared_encoding: Option<&str>,
) -> Result<(Kind, Vec<u8>), String> {
    let image = array
        .as_struct_opt()
        .ok_or_else(|| format!("expected a struct, got {}", array.data_type()))?;
    if image.len() != 1 {
        return Err(format!(
            "expected one image per message, got {}",
            image.len()
        ));
    }
    if image.is_null(0) {
        return Err("the image is null".to_string());
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
    if let Some(declared) = declared_encoding {
        let same = canonical_encoding(declared) == canonical_encoding(encoding)
            || compressed_format(declared).is_some_and(|f| compressed_format(encoding) == Some(f));
        if !same {
            return Err(format!(
                "the image's encoding `{encoding}` isn't the declared `{declared}`"
            ));
        }
    }
    let data = field("data")?;
    if let Some(format) = compressed_format(encoding) {
        let encoded = compressed_data(data.as_ref(), format)?;
        return Ok((
            Kind::CompressedImage,
            compressed_json(metadata, encoded, format)?,
        ));
    }
    let pixels = pixel_bytes(data.as_ref(), encoding)?;
    // The extra fields of a ROS `sensor_msgs/Image` (what dora's ROS 2 bridge
    // produces), when present.
    if let Some(step) = image.column_by_name("step") {
        let step = unsigned(step.as_ref(), "step")?;
        if step.checked_mul(rows(encoding, height)) != Some(pixels.len() as u64) {
            return Err(format!(
                "`step` {step} doesn't match {} bytes of {height} rows",
                pixels.len()
            ));
        }
    }
    if let Some(big_endian) = image.column_by_name("is_bigendian") {
        let big_endian = match big_endian.data_type() {
            DataType::Boolean if big_endian.is_null(0) => {
                return Err("`is_bigendian` is null".to_string());
            }
            DataType::Boolean => big_endian.as_boolean().value(0),
            _ => unsigned(big_endian.as_ref(), "is_bigendian")? != 0,
        };
        if big_endian && matches!(encoding, "mono16" | "16UC1" | "32FC1") {
            return Err(format!("big-endian `{encoding}` data isn't supported"));
        }
    }
    Ok((
        Kind::RawImage,
        raw_image(metadata, width, height, encoding, pixels, false)?,
    ))
}

/// Plain compressed-image bytes (binary or flat `uint8`) in a known format.
fn compressed_bytes(
    metadata: &Metadata,
    array: &ArrayRef,
    format: &str,
) -> Result<(Kind, Vec<u8>), String> {
    let encoded = compressed_data(array.as_ref(), format)?;
    Ok((
        Kind::CompressedImage,
        compressed_json(metadata, encoded, format)?,
    ))
}

fn metadata_str<'a>(metadata: &'a Metadata, key: &str) -> Option<&'a str> {
    match metadata.parameters.get(key) {
        Some(Parameter::String(value)) => Some(value),
        _ => None,
    }
}

fn metadata_int(metadata: &Metadata, key: &str) -> Option<i64> {
    match metadata.parameters.get(key) {
        Some(Parameter::Integer(value)) => Some(*value),
        _ => None,
    }
}

/// A frame sent as a flat array, the camera convention of the dora-hub nodes
/// (`opencv-video-capture`, `dora-pyrealsense`, `dora-pyorbbecksdk`, the
/// examples): compressed bytes when the encoding names JPEG/PNG/WebP/AVIF, or
/// raw pixels in one of the encodings Foxglove lists, with `width` and
/// `height` metadata. `encoding` overrides the `encoding` metadata.
///
/// `strict` is for undeclared outputs, where the metadata may just have been
/// forwarded from a camera onto data that isn't an image: a raw frame must be
/// exactly `width` pixels per row, without padding. (A compressed frame must
/// always start with the signature of its format.)
fn frame(
    metadata: &Metadata,
    encoding: Option<&str>,
    array: &ArrayRef,
    strict: bool,
) -> Result<(Kind, Vec<u8>), String> {
    let declared = encoding.is_some();
    let encoding = encoding
        .or_else(|| metadata_str(metadata, "encoding"))
        .ok_or_else(|| "expected an `Image` struct, or `encoding` metadata".to_string())?;
    if let Some(named) = compressed_format(encoding) {
        // A format on the declared type must match. One in the metadata (maybe
        // forwarded from another camera) only says the frame is compressed;
        // the file signature says which format.
        let format = if declared {
            named
        } else {
            encoded_bytes(array.as_ref())
                .and_then(detect_format)
                .ok_or_else(|| format!("data is not a {named} file"))?
        };
        return compressed_bytes(metadata, array, format);
    }
    let (Some(width), Some(height)) = (
        metadata_int(metadata, "width"),
        metadata_int(metadata, "height"),
    ) else {
        return Err(format!(
            "a raw `{encoding}` frame needs integer `width` and `height` metadata"
        ));
    };
    let pixels = pixel_bytes(array.as_ref(), encoding)?;
    let width = u64::try_from(width).map_err(|_| format!("negative width {width}"))?;
    let height = u64::try_from(height).map_err(|_| format!("negative height {height}"))?;
    Ok((
        Kind::RawImage,
        raw_image(metadata, width, height, encoding, pixels, strict)?,
    ))
}

/// Rows of `step` bytes in a frame: NV12 is a full-height Y plane followed by
/// a half-height UV plane.
fn rows(encoding: &str, height: u64) -> u64 {
    if encoding == "nv12" {
        height.saturating_add(height / 2)
    } else {
        height
    }
}

/// With `exact_rows`, rows must hold exactly `width` pixels (no padding).
fn raw_image(
    metadata: &Metadata,
    width: u64,
    height: u64,
    encoding: &str,
    pixels: &[u8],
    exact_rows: bool,
) -> Result<Vec<u8>, String> {
    let bytes_per_pixel = raw_bytes_per_pixel(encoding)
        .ok_or_else(|| format!("`{encoding}` is not an encoding Foxglove can display"))?;
    // Foxglove's RawImage has uint32 sizes; NV12 needs an even width and
    // height, and the YUV 4:2:2 encodings store pixels in pairs.
    if width == 0 || height == 0 || width > u64::from(u32::MAX) || height > u64::from(u32::MAX) {
        return Err(format!("a {width}x{height} image can't be displayed"));
    }
    let pairs = matches!(
        encoding,
        "nv12" | "yuv422" | "uyvy" | "yuv422_yuy2" | "yuyv"
    );
    if pairs && (!width.is_multiple_of(2) || (encoding == "nv12" && !height.is_multiple_of(2))) {
        return Err(format!(
            "`{encoding}` needs even dimensions, got {width}x{height}"
        ));
    }
    let rows = usize::try_from(rows(encoding, height))
        .map_err(|_| format!("height {height} is too large"))?;
    if rows == 0 || !pixels.len().is_multiple_of(rows) {
        return Err(format!(
            "{} bytes of `{encoding}` data are not whole rows of a {width}x{height} image",
            pixels.len()
        ));
    }
    let step = pixels.len() / rows;
    let row_bytes = width.saturating_mul(bytes_per_pixel);
    if (step as u64) < row_bytes {
        return Err(format!(
            "rows of {step} bytes cannot hold {width} `{encoding}` pixels"
        ));
    }
    if exact_rows && step as u64 != row_bytes {
        return Err(format!(
            "rows of {step} bytes are not {width} `{encoding}` pixels"
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

/// One name per encoding, for the pairs Foxglove's RawImage lists as the same.
fn canonical_encoding(encoding: &str) -> &str {
    match encoding {
        "8UC1" => "mono8",
        "8UC3" => "bgr8",
        "16UC1" => "mono16",
        "uyvy" => "yuv422",
        "yuyv" => "yuv422_yuy2",
        other => other,
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

/// A declared `std/media/v1/CompressedImage`: one binary value, or a struct
/// with a `data` field like ROS `sensor_msgs/CompressedImage`. The format is
/// the type's `format` parameter (the data must be that format), else the
/// data's file signature.
fn compressed_image(
    metadata: &Metadata,
    array: &ArrayRef,
    declared_format: Option<&str>,
) -> Result<Vec<u8>, String> {
    let data = match array.as_struct_opt() {
        Some(image) => {
            if image.len() != 1 {
                return Err(format!(
                    "expected one image per message, got {}",
                    image.len()
                ));
            }
            if image.is_null(0) {
                return Err("the image is null".to_string());
            }
            image
                .column_by_name("data")
                .ok_or_else(|| "image struct has no `data` field".to_string())?
                .clone()
        }
        None => array.clone(),
    };
    let format = match declared_format {
        Some(name) => compressed_format(name)
            .ok_or_else(|| format!("`{name}` is not a format Foxglove can display"))?,
        None => encoded_bytes(data.as_ref())
            .and_then(detect_format)
            .ok_or_else(|| {
                "unknown compressed image format; declare it as `std/media/v1/CompressedImage[format=jpeg]`"
                    .to_string()
            })?,
    };
    compressed_json(metadata, compressed_data(data.as_ref(), format)?, format)
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

/// Compressed image bytes (binary or flat `uint8`) that really are `format`,
/// checked by file signature: anything else isn't exported as an image.
fn compressed_data<'a>(array: &'a dyn Array, format: &str) -> Result<&'a [u8], String> {
    let encoded = encoded_bytes(array)
        .ok_or_else(|| format!("expected binary `{format}` data, got {}", array.data_type()))?;
    match detect_format(encoded) {
        Some(detected) if detected == format => Ok(encoded),
        _ => Err(format!("data is not a {format} file")),
    }
}

/// The formats Foxglove's `CompressedImage` accepts, by file signature.
fn detect_format(data: &[u8]) -> Option<&'static str> {
    if data.starts_with(&[0xFF, 0xD8, 0xFF]) {
        Some("jpeg")
    } else if data.starts_with(&[0x89, b'P', b'N', b'G', 0x0D, 0x0A, 0x1A, 0x0A]) {
        Some("png")
    } else if data.len() >= 12 && &data[..4] == b"RIFF" && &data[8..12] == b"WEBP" {
        Some("webp")
    } else if is_avif(data) {
        Some("avif")
    } else {
        None
    }
}

/// An ISO-BMFF file whose `ftyp` box lists the `avif` (still) or `avis`
/// (sequence) brand, as major or compatible brand.
fn is_avif(data: &[u8]) -> bool {
    if data.len() < 12 || &data[4..8] != b"ftyp" {
        return false;
    }
    let size = u32::from_be_bytes([data[0], data[1], data[2], data[3]]) as usize;
    let end = size.min(data.len()).max(16.min(data.len()));
    // Major brand at 8..12, minor version at 12..16, compatible brands after.
    std::iter::once(&data[8..12])
        .chain(data.get(16..end).unwrap_or_default().chunks_exact(4))
        .any(|brand| brand == b"avif" || brand == b"avis")
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
        DataType::List(item) | DataType::LargeList(item)
            if array.len() == 1 && item.data_type() == &DataType::UInt8 =>
        {
            let (values, start, end) = match array.data_type() {
                DataType::List(_) => {
                    let list = array.as_list::<i32>();
                    let offsets = list.value_offsets();
                    (list.values(), offsets[0] as usize, offsets[1] as usize)
                }
                _ => {
                    let list = array.as_list::<i64>();
                    let offsets = list.value_offsets();
                    (list.values(), offsets[0] as usize, offsets[1] as usize)
                }
            };
            let values = values.as_primitive::<UInt8Type>();
            (values.null_count() == 0).then(|| &values.values()[start..end])
        }
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

/// The bytes of raw pixel data, if the array's element type fits `encoding`:
/// bytes (binary or `uint8`) for any encoding, as in a ROS `Image`, and also
/// `uint16`/`int16` for the 16-bit single-channel encodings and `float32` for
/// `32FC1`. A float32 array with `rgb8` metadata, say, is not an image.
fn pixel_bytes<'a>(array: &'a dyn Array, encoding: &str) -> Result<&'a [u8], String> {
    let fits = match array.data_type() {
        DataType::Binary | DataType::LargeBinary | DataType::UInt8 => true,
        DataType::List(item) | DataType::LargeList(item) => item.data_type() == &DataType::UInt8,
        DataType::UInt16 | DataType::Int16 => matches!(encoding, "mono16" | "16UC1"),
        DataType::Float32 => encoding == "32FC1",
        _ => false,
    };
    if !fits {
        return Err(format!(
            "{} data doesn't fit encoding `{encoding}`",
            array.data_type()
        ));
    }
    bytes(array).ok_or_else(|| format!("expected a flat image array, got {}", array.data_type()))
}

/// The bytes of a compressed image: one binary value or a flat `uint8` array.
/// Wider numeric arrays can't hold a JPEG/PNG/WebP/AVIF file.
fn encoded_bytes(array: &dyn Array) -> Option<&[u8]> {
    match array.data_type() {
        DataType::Binary | DataType::LargeBinary | DataType::UInt8 => bytes(array),
        DataType::List(item) | DataType::LargeList(item)
            if item.data_type() == &DataType::UInt8 =>
        {
            bytes(array)
        }
        _ => None,
    }
}

fn unsigned(array: &dyn Array, name: &str) -> Result<u64, String> {
    use arrow::datatypes::{
        Int8Type, Int16Type, Int32Type, Int64Type, UInt8Type, UInt16Type, UInt32Type, UInt64Type,
    };
    if array.is_empty() || array.is_null(0) {
        return Err(format!("`{name}` is null"));
    }
    let signed = |value: i64| u64::try_from(value).map_err(|_| format!("negative `{name}`"));
    match array.data_type() {
        DataType::UInt8 => Ok(u64::from(array.as_primitive::<UInt8Type>().value(0))),
        DataType::UInt16 => Ok(u64::from(array.as_primitive::<UInt16Type>().value(0))),
        DataType::UInt32 => Ok(u64::from(array.as_primitive::<UInt32Type>().value(0))),
        DataType::UInt64 => Ok(array.as_primitive::<UInt64Type>().value(0)),
        DataType::Int8 => signed(i64::from(array.as_primitive::<Int8Type>().value(0))),
        DataType::Int16 => signed(i64::from(array.as_primitive::<Int16Type>().value(0))),
        DataType::Int32 => signed(i64::from(array.as_primitive::<Int32Type>().value(0))),
        DataType::Int64 => signed(array.as_primitive::<Int64Type>().value(0)),
        other => Err(format!("`{name}` must be an integer, got {other}")),
    }
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
    use super::{Kind, Mapper, Recognition, detect_format};

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

    /// An undeclared frame that has camera metadata but doesn't convert says
    /// why, so the warning for a topic that is already an image channel names
    /// the real problem.
    #[test]
    fn an_undeclared_frame_that_fails_keeps_its_reason() {
        use arrow::array::UInt8Array;
        use dora_message::metadata::{Metadata, Parameter};
        // Claimed to be a JPEG, but not any image at all.
        let png = UInt8Array::from(vec![1u8, 2, 3, 4, 5]);
        let mut payload = Vec::new();
        {
            let batch = arrow::record_batch::RecordBatch::try_from_iter([(
                "c",
                std::sync::Arc::new(png) as arrow::array::ArrayRef,
            )])
            .unwrap();
            let mut writer =
                arrow::ipc::writer::StreamWriter::try_new(&mut payload, &batch.schema()).unwrap();
            writer.write(&batch).unwrap();
            writer.finish().unwrap();
        }
        let mut metadata = Metadata::new(dora_message::uhlc::HLC::default().new_timestamp());
        metadata
            .parameters
            .insert("encoding".into(), Parameter::String("jpeg".into()));
        let mapper = Mapper::from_descriptor_yaml("nodes: []");
        match mapper.convert("cam", "image", &metadata, Some(&payload)) {
            Recognition::NotRecognised(Some(reason)) => {
                assert!(reason.contains("not a jpeg file"), "{reason}")
            }
            _ => panic!("expected NotRecognised with a reason"),
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
        // `mif1` major brand with `avif` among the compatible brands.
        let mut mif1 = vec![0, 0, 0, 0x18];
        mif1.extend(b"ftypmif1\0\0\0\0mif1avif");
        assert_eq!(detect_format(&mif1), Some("avif"));
        // HEIC is ISO-BMFF too, but not a format Foxglove's CompressedImage takes.
        let mut heic = vec![0, 0, 0, 0x18];
        heic.extend(b"ftypheic\0\0\0\0mif1heic");
        assert_eq!(detect_format(&heic), None);
    }
}
