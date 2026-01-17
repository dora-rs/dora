use dora_cli::{build, run};
use eyre::Context;
use std::path::Path;

fn main() -> eyre::Result<()> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    build("dataflow.yml".to_string(), None, None, false, true)?;

    run("dataflow.yml".to_string(), false, None)?;

    Ok(())
}
