use dora_cli::{build, run as dora_run};
use eyre::WrapErr;
use std::path::Path;

fn main() -> eyre::Result<()> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    build("dataflow.yml".to_string(), None, None, true, true)?;

    dora_run("dataflow.yml".to_string(), true, None)?;

    Ok(())
}
