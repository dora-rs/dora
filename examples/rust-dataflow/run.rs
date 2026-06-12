use dora_cli::{BuildConfig, Executable, RunCommand, build};
use eyre::Context;
use std::{path::Path, time::Duration};

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

    build(BuildConfig {
        dataflow: dataflow.clone(),
        force_local: true,
        ..Default::default()
    })?;

    // Bound the run so a wedged node fails fast via the daemon's stop
    // escalation instead of hanging until the CI step timeout (#2152).
    // A healthy run self-terminates quickly.
    let mut run = RunCommand::new(dataflow);
    run.stop_after = Some(Duration::from_secs(120));
    run.execute()?;

    Ok(())
}
