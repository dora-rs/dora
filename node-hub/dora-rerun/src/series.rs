use dora_node_api::{
    into_vec_f64,
    dora_core::config::DataId,
    ArrowData,
};
use eyre::{Context, Result};
use rerun::RecordingStream;

pub fn update_series(rec: &RecordingStream, id: DataId, data: ArrowData) -> Result<()> {
    let series = into_vec_f64(&data).context("could not cast values")?;
    for (i, value) in series.iter().enumerate() {
        rec.log(
            format!("{}_{}", id.as_str(), i),
            &rerun::Scalar::new(*value as f64),
        )
        .wrap_err("could not log series")?;
    }
    Ok(())
}
