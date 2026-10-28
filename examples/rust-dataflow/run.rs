use dora_cli::{BuildConfig, Executable, RunCommand, build};
use eyre::{Context, bail};
use std::{
    path::Path,
    time::{Duration, Instant},
};

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

    // Bound the run so a wedged node can't hang CI (#2152). A node the daemon
    // kills with SIGTERM at the deadline still counts as a clean stop, and a
    // healthy run ends long before it, so check the time after `execute()`.
    let deadline = Duration::from_secs(120);
    let mut run = RunCommand::new(dataflow);
    run.stop_after = Some(deadline);
    let started = Instant::now();
    run.execute()?;
    if started.elapsed() >= deadline {
        bail!("the dataflow did not finish within {deadline:?}, so a node hung");
    }

    Ok(())
}
