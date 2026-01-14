use dora_cli::{build, run};
use eyre::Context;
use std::path::Path;

fn main() -> eyre::Result<()> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .wrap_err("failed to set working dir")?;

    let args: Vec<String> = std::env::args().collect();
    let dataflow = if args.len() > 1 {
        args[1].clone()
    } else {
        "dataflow.yml".to_string()
    };

    build(dataflow.clone(), None, None, false, true)?;

    run(dataflow, false, None)?;

    Ok(())
}
