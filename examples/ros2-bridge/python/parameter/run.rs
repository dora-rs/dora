use dora_cli::{BuildConfig, build, run as dora_run};
use eyre::WrapErr;
use std::path::Path;

fn main() -> eyre::Result<()> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../");
    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    build(BuildConfig {
        dataflow: "dataflow.yml".to_string(),
        uv: true,
        force_local: true,
        ..Default::default()
    })?;

    // Boots the parameter server and runs the tick loop. With no external ROS2
    // peer (host CI has no RMW) this proves construction + spin_once don't
    // panic; the real `ros2 param` round-trip is asserted in scripts/ros2dev.sh.
    dora_run("dataflow.yml".to_string(), true)?;

    Ok(())
}
