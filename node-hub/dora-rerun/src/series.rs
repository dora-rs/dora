use dora_node_api::{ArrowData, dora_core::config::DataId, into_vec};
use eyre::{Context, Result};
use rerun::RecordingStream;

pub fn update_series(rec: &RecordingStream, id: DataId, data: ArrowData) -> Result<()> {
    let series = into_vec::<f64>(&data).context("could not cast values")?;
    for (i, value) in series.iter().enumerate() {
        rec.log(
            format!("{}_{}", id.as_str(), i),
            &rerun::Scalars::new([*value]),
        )
        .wrap_err("could not log series")?;
    }
    Ok(())
}
