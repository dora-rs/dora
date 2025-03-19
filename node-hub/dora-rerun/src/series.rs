use dora_node_api::{
    arrow::array::{Float32Array, Float64Array, Int32Array, Int64Array, UInt32Array},
    arrow::datatypes::DataType,
    dora_core::config::DataId,
    ArrowData,
};
use eyre::{Context, ContextCompat, Result};
use rerun::RecordingStream;

macro_rules! register_array_handlers {
    ($(($variant:path, $array_type:ty, $type_name:expr)),* $(,)?) => {
        pub fn update_series(rec: &RecordingStream, id: DataId, data: ArrowData) -> Result<()> {
            match data.data_type() {
                $(
                    $variant => {
                        let buffer: &$array_type = data
                            .as_any()
                            .downcast_ref()
                            .context(concat!("series is not ", $type_name))?;
                        let series: Vec<_> = buffer.values().to_vec();
                        for (i, value) in series.iter().enumerate() {
                            rec.log(
                                format!("{}_{}", id.as_str(), i),
                                &rerun::Scalar::new(*value as f64),
                            )
                            .wrap_err("could not log series")?;
                        }
                    }
                ),*
                _ => unimplemented!("This has not yet implemented. Please contribute to dora-rerun :)"),
            }
            Ok(())
        }
    };
}

// Register all supported array types in one place
register_array_handlers! {
    (DataType::Float32, Float32Array, "float32"),
    (DataType::Float64, Float64Array, "float64"),
    (DataType::Int32, Int32Array, "int32"),
    (DataType::Int64, Int64Array, "int64"),
    // Add new types here easily:
    (DataType::UInt32, UInt32Array, "uint32"),
}
