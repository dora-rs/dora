use dora_cli::{BuildConfig, build, run as dora_run};
use eyre::WrapErr;
use std::path::Path;

fn main() -> eyre::Result<()> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    build(BuildConfig {
        dataflow: "dataflow.yml".to_string(),
        uv: true,
        force_local: true,
        ..Default::default()
    })?;

    dora_run("dataflow.yml".to_string(), true)?;

    Ok(())
}
