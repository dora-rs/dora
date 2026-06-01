use dora_cli::{BuildConfig, build, run};
use eyre::Context;
use std::path::Path;

fn main() -> eyre::Result<()> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    std::env::set_current_dir(root.join("../../../").join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    build(BuildConfig {
        dataflow: "dataflow.yml".to_string(),
        force_local: true,
        ..Default::default()
    })?;

    // Self-contained: the dataflow runs both the dora Fibonacci action server and
    // a dora action client (both ros2-client, so they discover each other and use
    // a consistent CDR). The client sends one goal; the server serves it and both
    // nodes exit. No external ROS2 process is required.
    run("dataflow.yml".to_string(), false)?;
    Ok(())
}
