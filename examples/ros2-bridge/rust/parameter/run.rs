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

    // The parameter example is self-contained: the node declares and exercises
    // its parameters via the local API and exits, so there is no external ROS2
    // client to pair with.
    run("dataflow.yml".to_string(), false)
}
