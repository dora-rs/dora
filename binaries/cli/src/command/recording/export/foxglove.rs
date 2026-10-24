//! `dora recording export --schema foxglove`: re-encode image outputs as
//! Foxglove's well-known `foxglove.RawImage` / `foxglove.CompressedImage`
//! schemas, JSON-encoded, so Foxglove can display them. The Arrow channels
//! the default export writes are valid MCAP but Foxglove does not render
//! Arrow (#3489, tier 2).
//!
//! An output is an image when the dataflow says so: `output_types` in the
//! descriptor stored in the recording header declares `std/media/v1/Image` or
//! `std/media/v1/CompressedImage`. The only undeclared outputs converted are
//! compressed frames: `encoding` metadata naming JPEG/PNG/WebP, and bytes that
//! start with one of those file signatures. Nodes often forward a camera's
//! metadata onto outputs that are not images, so metadata alone never makes
//! an image, and undeclared raw pixels stay Arrow.
use std::cell::Cell;
use std::collections::{BTreeMap, HashMap};

use arrow::array::{Array, ArrayRef, AsArray, StructArray};
use arrow::datatypes::{DataType, UInt8Type};
use arrow::ipc::reader::StreamDecoder;
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
    /// Not an image: export it as Arrow like the default schema does.
    NotRecognised,
    /// The JSON message for this Foxglove schema.
    Converted(Kind, Vec<u8>),
    /// A declared image output, but this message could not be converted.
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
    /// just declares nothing: export still works, converting only
    /// undeclared compressed frames.
    pub(super) fn from_descriptor_yaml(yaml: &str) -> Self {
        let mut declared: HashMap<String, HashMap<String, Declared>> = HashMap::new();
        let Ok(descriptor) = serde_yaml::from_str::<DescriptorTypes>(yaml) else {
            return Self { declared };
        };
        for node in descriptor.nodes {
            let outputs = declared.entry(node.id).or_default();
            add_declared(outputs, node.output_types, None);
            // `dora record` and `--proxy` both record a single operator's
            // outputs without the operator id (`record.rs`
            // `discover_descriptor_outputs`, `public_topic_output_id`).
            if let Some(operator) = node.operator {
                add_declared(outputs, operator.output_types, None);
            }
            for operator in node.operators {
                if let Some(operator_id) = operator.id {
                    add_declared(outputs, operator.output_types, Some(&operator_id));
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
        // Recordings have no checksum, and arrow-ipc panics on many kinds of
        // damaged payload (bad schema bits, impossible lengths, offsets out
        // of range). A damaged message is then one that can't be converted,
        // like any other; `--schema arrow` copies it without decoding.
        quiet_conversion_panics();
        CONVERTING.set(true);
        let converted = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            convert_unguarded(declared, metadata, data)
        }));
        CONVERTING.set(false);
        converted.unwrap_or_else(|_| {
            let reason = "the payload is damaged (not a valid Arrow IPC stream)".to_string();
            match declared {
                Some(Declared::Image { encoding }) => {
                    Recognition::Failed(expected_kind(encoding.as_deref(), metadata), reason)
                }
                Some(Declared::CompressedImage { .. }) => {
                    Recognition::Failed(Kind::CompressedImage, reason)
                }
                Some(Declared::Other) | None => Recognition::NotRecognised,
            }
        })
    }
}

thread_local! {
    /// Set while this thread converts a message, so the panic hook stays
    /// quiet for the panics [`Mapper::convert`] catches.
    static CONVERTING: Cell<bool> = const { Cell::new(false) };
}

/// A caught panic is already reported as a dropped message, so the default
/// hook's `thread 'main' panicked at ...` for each damaged message would only
/// make the export look like a crash. Installed once; every other panic still
/// goes to the previous hook.
fn quiet_conversion_panics() {
    static INSTALL: std::sync::Once = std::sync::Once::new();
    INSTALL.call_once(|| {
        let previous = std::panic::take_hook();
        std::panic::set_hook(Box::new(move |info| {
            if !CONVERTING.get() {
                previous(info);
            }
        }));
    });
}

fn convert_unguarded(
    declared: Option<&Declared>,
    metadata: &Metadata,
    data: Option<&[u8]>,
) -> Recognition {
    match declared {
        Some(Declared::Image { encoding }) => {
            // Declared images come as the `Image` struct, or as a flat
            // array: compressed bytes, or pixels with width/height
            // metadata. An encoding on the type (`Image[encoding=jpeg]`)
            // wins over one the node forwarded in its metadata.
            let encoding = encoding.as_deref();
            let result = decode(data).and_then(|array| match array.as_struct_opt() {
                Some(image) => image_struct(metadata, image),
                None => frame(metadata, encoding, &array),
            });
            match result {
                Ok((kind, json)) => Recognition::Converted(kind, json),
                Err(reason) => Recognition::Failed(expected_kind(encoding, metadata), reason),
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
        Some(Declared::Other) => Recognition::NotRecognised,
        // Not declared: only a compressed frame whose bytes really are the
        // file the metadata says. The metadata is checked first, so other
        // outputs aren't decoded. The signature, not the (maybe forwarded)
        // name, decides the format.
        None => {
            if metadata_str(metadata, "encoding")
                .and_then(compressed_format)
                .is_none()
            {
                return Recognition::NotRecognised;
            }
            let converted = decode(data).ok().and_then(|array| {
                let encoded = encoded_bytes(array.as_ref())?;
                compressed_json(metadata, encoded, detect_format(encoded)?).ok()
            });
            match converted {
                Some(json) => Recognition::Converted(Kind::CompressedImage, json),
                None => Recognition::NotRecognised,
            }
        }
    }
}

/// The kind a declared `Image` is expected to convert to, for the report
/// when it fails.
fn expected_kind(declared_encoding: Option<&str>, metadata: &Metadata) -> Kind {
    match declared_encoding
        .or_else(|| metadata_str(metadata, "encoding"))
        .and_then(compressed_format)
    {
        Some(_) => Kind::CompressedImage,
        None => Kind::RawImage,
    }
}

/// The parts of the dataflow descriptor this export reads. Ids are
/// deserialized as strings, which keeps their YAML text the way dora reads
/// them: `0x10:` is output `0x10`, not `16`.
#[derive(serde::Deserialize)]
struct DescriptorTypes {
    #[serde(default)]
    nodes: Vec<NodeTypes>,
}

#[derive(serde::Deserialize)]
struct NodeTypes {
    id: String,
    #[serde(default)]
    output_types: BTreeMap<String, serde_yaml::Value>,
    operator: Option<OperatorTypes>,
    #[serde(default)]
    operators: Vec<OperatorTypes>,
}

#[derive(serde::Deserialize)]
struct OperatorTypes {
    id: Option<String>,
    #[serde(default)]
    output_types: BTreeMap<String, serde_yaml::Value>,
}

/// Records the image types in `output_types`, keyed by output id, prefixed
/// with `operator_id/` for an operator.
fn add_declared(
    outputs: &mut HashMap<String, Declared>,
    output_types: BTreeMap<String, serde_yaml::Value>,
    operator_id: Option<&str>,
) {
    for (output_id, urn) in output_types {
        let key = match operator_id {
            Some(operator_id) => format!("{operator_id}/{output_id}"),
            None => output_id,
        };
        // Full URNs only: `dora validate` does not resolve short names in
        // `output_types` either. Anything else, a malformed type included,
        // still declares the output as not an image.
        let image = match urn.as_str().and_then(dora_core::types::parse_urn) {
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
/// `encoding`, `data` (binary). A compressed `encoding` (jpeg, png, webp)
/// makes it a `CompressedImage`.
fn image_struct(metadata: &Metadata, image: &StructArray) -> Result<(Kind, Vec<u8>), String> {
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
    if encoding_column.is_null(0) {
        return Err("`encoding` is null".to_string());
    }
    let encoding = match encoding_column.data_type() {
        DataType::Utf8 => encoding_column.as_string::<i32>().value(0),
        DataType::LargeUtf8 => encoding_column.as_string::<i64>().value(0),
        other => return Err(format!("`encoding` must be a string, got {other}")),
    };
    let data = field("data")?;
    if let Some(format) = compressed_format(encoding) {
        let encoded = compressed_data(data.as_ref(), format)?;
        return Ok((
            Kind::CompressedImage,
            compressed_json(metadata, encoded, format)?,
        ));
    }
    let pixels = pixel_bytes(data.as_ref(), encoding)?;
    Ok((
        Kind::RawImage,
        raw_image(metadata, width, height, encoding, pixels)?,
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

/// A declared image sent as a flat array, the camera convention of the
/// dora-hub nodes (`opencv-video-capture`, `dora-pyrealsense`, the examples):
/// compressed bytes when the encoding names JPEG/PNG/WebP, or raw pixels with
/// `width` and `height` metadata. `encoding` (from the declared type)
/// overrides the `encoding` metadata.
fn frame(
    metadata: &Metadata,
    encoding: Option<&str>,
    array: &ArrayRef,
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
        let encoded = compressed_data(array.as_ref(), format)?;
        return Ok((
            Kind::CompressedImage,
            compressed_json(metadata, encoded, format)?,
        ));
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
        raw_image(metadata, width, height, encoding, pixels)?,
    ))
}

/// Rows must be exactly `width` pixels: padding can't be told apart from
/// stale or forwarded dimensions.
fn raw_image(
    metadata: &Metadata,
    width: u64,
    height: u64,
    encoding: &str,
    pixels: &[u8],
) -> Result<Vec<u8>, String> {
    let bytes_per_pixel = raw_bytes_per_pixel(encoding)
        .ok_or_else(|| format!("`{encoding}` is not a raw encoding this export supports"))?;
    // Foxglove's RawImage has uint32 sizes.
    if width == 0 || height == 0 || width > u64::from(u32::MAX) || height > u64::from(u32::MAX) {
        return Err(format!("a {width}x{height} image can't be displayed"));
    }
    let step = width * bytes_per_pixel;
    if step.checked_mul(height) != Some(pixels.len() as u64) {
        return Err(format!(
            "{} bytes of `{encoding}` data are not a {width}x{height} image",
            pixels.len()
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

/// Bytes per pixel for the single-plane raw encodings Foxglove's `RawImage`
/// lists; `None` for anything else.
fn raw_bytes_per_pixel(encoding: &str) -> Option<u64> {
    match encoding {
        "mono8" | "8UC1" | "bayer_rggb8" | "bayer_bggr8" | "bayer_gbrg8" | "bayer_grbg8" => Some(1),
        "mono16" | "16UC1" => Some(2),
        "rgb8" | "bgr8" | "8UC3" => Some(3),
        "rgba8" | "bgra8" | "32FC1" => Some(4),
        _ => None,
    }
}

/// The `foxglove.CompressedImage` format for an encoding name, if it is one.
fn compressed_format(encoding: &str) -> Option<&'static str> {
    match encoding {
        "jpeg" => Some("jpeg"),
        "png" => Some("png"),
        "webp" => Some("webp"),
        _ => None,
    }
}

/// A declared `std/media/v1/CompressedImage`: one binary value or a flat
/// `uint8` array. The format is the type's `format` parameter (the data must
/// be that format), else the data's file signature.
fn compressed_image(
    metadata: &Metadata,
    array: &ArrayRef,
    declared_format: Option<&str>,
) -> Result<Vec<u8>, String> {
    let format = match declared_format {
        Some(name) => compressed_format(name)
            .ok_or_else(|| format!("`{name}` is not a format this export supports"))?,
        None => encoded_bytes(array.as_ref())
            .and_then(detect_format)
            .ok_or_else(|| {
                "unknown compressed image format; declare it as `std/media/v1/CompressedImage[format=jpeg]`"
                    .to_string()
            })?,
    };
    compressed_json(metadata, compressed_data(array.as_ref(), format)?, format)
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

/// The supported `CompressedImage` formats, by file signature.
fn detect_format(data: &[u8]) -> Option<&'static str> {
    if data.starts_with(&[0xFF, 0xD8, 0xFF]) {
        Some("jpeg")
    } else if data.starts_with(&[0x89, b'P', b'N', b'G', 0x0D, 0x0A, 0x1A, 0x0A]) {
        Some("png")
    } else if data.len() >= 12 && &data[..4] == b"RIFF" && &data[8..12] == b"WEBP" {
        Some("webp")
    } else {
        None
    }
}

/// The single array a dora output carries: column 0 of the one record batch
/// in its Arrow IPC payload.
///
/// `StreamDecoder` only ever takes the bytes that are there, so a corrupt
/// `bodyLength` can't make it allocate that much (`StreamReader` allocates it
/// up front). Other damage that makes arrow-ipc panic is caught in
/// [`Mapper::convert`].
fn decode(data: Option<&[u8]>) -> Result<ArrayRef, String> {
    let data = data.ok_or_else(|| "message has no payload".to_string())?;
    let invalid =
        |err: arrow::error::ArrowError| format!("payload is not an Arrow IPC stream: {err}");
    let mut decoder = StreamDecoder::new();
    let mut buffer = arrow::buffer::Buffer::from(data);
    let mut batches = Vec::new();
    while !buffer.is_empty() {
        if let Some(batch) = decoder.decode(&mut buffer).map_err(invalid)? {
            batches.push(batch);
        }
    }
    decoder.finish().map_err(invalid)?;
    let batch = match <[_; 1]>::try_from(batches) {
        Ok([batch]) => batch,
        Err(batches) if batches.is_empty() => return Err("payload has no record batch".to_string()),
        Err(_) => return Err("payload has more than one record batch".to_string()),
    };
    if batch.num_columns() != 1 {
        return Err(format!("expected one column, got {}", batch.num_columns()));
    }
    Ok(batch.column(0).clone())
}

/// The bytes of a one-value binary array, or the value buffer of a flat
/// array without nulls: `uint8` pixels, `uint16` (e.g. `mono16` depth) or
/// `float32` (`32FC1`). Arrow buffers are little-endian on every
/// platform dora ships for, matching what Foxglove expects.
fn bytes(array: &dyn Array) -> Option<&[u8]> {
    use arrow::datatypes::{Float32Type, UInt16Type};
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
/// bytes (binary or `uint8`) for any encoding, and also `uint16` for the
/// 16-bit single-channel encodings and `float32` for `32FC1`. A float32
/// array with `rgb8` metadata, say, is not an image.
fn pixel_bytes<'a>(array: &'a dyn Array, encoding: &str) -> Result<&'a [u8], String> {
    let fits = match array.data_type() {
        DataType::Binary | DataType::LargeBinary | DataType::UInt8 => true,
        DataType::UInt16 => matches!(encoding, "mono16" | "16UC1"),
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
/// Wider numeric arrays can't hold a JPEG/PNG/WebP file.
fn encoded_bytes(array: &dyn Array) -> Option<&[u8]> {
    match array.data_type() {
        DataType::Binary | DataType::LargeBinary | DataType::UInt8 => bytes(array),
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
    use super::{Kind, Mapper, detect_format};

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

    fn ipc(array: arrow::array::ArrayRef) -> Vec<u8> {
        let batch = arrow::record_batch::RecordBatch::try_from_iter([("c", array)]).unwrap();
        let mut payload = Vec::new();
        let mut writer =
            arrow::ipc::writer::StreamWriter::try_new(&mut payload, &batch.schema()).unwrap();
        writer.write(&batch).unwrap();
        writer.finish().unwrap();
        drop(writer);
        payload
    }

    /// Recordings have no checksum, and arrow-ipc panics on many kinds of
    /// damaged payload (bad schema bits, impossible lengths, offsets out of
    /// range). Every message converts or fails cleanly: damage one byte at a
    /// time in valid payloads and check that nothing panics.
    #[test]
    fn damaged_payloads_never_panic() {
        use arrow::array::{
            ArrayRef, BinaryArray, StringArray, StructArray, UInt8Array, UInt16Array,
        };
        use arrow_schema::{DataType, Field};
        use dora_message::metadata::{Metadata, Parameter};
        use std::sync::Arc;
        let image = Arc::new(StructArray::from(vec![
            (
                Arc::new(Field::new("width", DataType::UInt16, false)),
                Arc::new(UInt16Array::from(vec![2])) as ArrayRef,
            ),
            (
                Arc::new(Field::new("height", DataType::UInt16, false)),
                Arc::new(UInt16Array::from(vec![1])) as ArrayRef,
            ),
            (
                Arc::new(Field::new("encoding", DataType::Utf8, false)),
                Arc::new(StringArray::from(vec!["mono8"])) as ArrayRef,
            ),
            (
                Arc::new(Field::new("data", DataType::Binary, false)),
                Arc::new(BinaryArray::from(vec![[1u8, 2].as_slice()])) as ArrayRef,
            ),
        ])) as ArrayRef;
        let payloads = [
            ipc(Arc::new(UInt8Array::from(vec![
                Some(1u8),
                None,
                Some(3),
                Some(4),
            ]))),
            ipc(Arc::new(UInt16Array::from(vec![1u16, 2]))),
            ipc(Arc::new(BinaryArray::from(vec![
                [0xFFu8, 0xD8, 0xFF, 0xE0].as_slice(),
            ]))),
            ipc(image),
        ];
        let mut raw = Metadata::new(dora_message::uhlc::HLC::default().new_timestamp());
        for (key, value) in [
            ("width", Parameter::Integer(2)),
            ("height", Parameter::Integer(2)),
            ("encoding", Parameter::String("mono8".into())),
        ] {
            raw.parameters.insert(key.into(), value);
        }
        // Undeclared outputs are only decoded with a compressed encoding.
        let mut jpeg = raw.clone();
        jpeg.parameters
            .insert("encoding".into(), Parameter::String("jpeg".into()));
        let mapper = Mapper::from_descriptor_yaml(
            "nodes:\n  - id: cam\n    output_types:\n      image: std/media/v1/Image\n",
        );
        for payload in &payloads {
            for at in 0..payload.len() {
                for flip in [0x01u8, 0x80, 0xFF] {
                    let mut damaged = payload.clone();
                    damaged[at] ^= flip;
                    for (output, metadata) in [("image", &raw), ("other", &jpeg)] {
                        let result = std::panic::catch_unwind(|| {
                            mapper.convert("cam", output, metadata, Some(&damaged));
                        });
                        assert!(
                            result.is_ok(),
                            "byte {at} ^ {flip:#x} panicked on `{output}`"
                        );
                    }
                }
            }
        }
        // A caught panic must not leave the hook quiet for later panics.
        assert!(!super::CONVERTING.get());
    }

    #[test]
    fn detects_the_formats_this_export_supports() {
        assert_eq!(detect_format(&[0xFF, 0xD8, 0xFF, 0xDB]), Some("jpeg"));
        assert_eq!(
            detect_format(&[0x89, b'P', b'N', b'G', 0x0D, 0x0A, 0x1A, 0x0A]),
            Some("png")
        );
        assert_eq!(detect_format(b"RIFF\0\0\0\0WEBPVP8 "), Some("webp"));
        assert_eq!(detect_format(b"GIF89a"), None);
        assert_eq!(detect_format(b"RIFF\0\0\0\0WAVE"), None);
    }
}
