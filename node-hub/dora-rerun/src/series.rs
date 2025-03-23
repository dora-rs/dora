use dora_node_api::{dora_core::config::DataId, into_vec_f64, ArrowData};
use eyre::{Context, Result};
use rerun::RecordingStream;

pub fn update_series(rec: &RecordingStream, id: DataId, data: ArrowData) -> Result<()> {
    let series = into_vec_f64(&data).context("could not cast values")?;
    for (i, value) in series.iter().enumerate() {
        rec.log(
            format!("{}_{}", id.as_str(), i),
            &rerun::Scalar::new(*value),
        )
        .wrap_err("could not log series")?;
    }
    Ok(())
}
