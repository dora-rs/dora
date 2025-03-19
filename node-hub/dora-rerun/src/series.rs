use dora_node_api::{
    arrow::array::{Float32Array, Float64Array, Int32Array, Int64Array, UInt32Array},
    arrow::datatypes::DataType,
    cast_values,
    dora_core::config::DataId,
    ArrowData,
};
use eyre::{Context, ContextCompat, Result};
use rerun::RecordingStream;

pub fn update_series(rec: &RecordingStream, id: DataId, data: ArrowData) -> Result<()> {
    let series = cast_values(&data).context("could not cast values")?;
    for (i, value) in series.iter().enumerate() {
        rec.log(
            format!("{}_{}", id.as_str(), i),
            &rerun::Scalar::new(*value as f64),
        )
        .wrap_err("could not log series")?;
    }
    Ok(())
}
