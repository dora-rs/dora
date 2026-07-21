use dora_cli::{BuildConfig, Executable, RunCommand, build};
use eyre::{Context, bail};
use std::{
    path::Path,
    time::{Duration, Instant},
};

/// Backstop so a wedged dataflow fails instead of hanging until the CI step
/// timeout (#2089).
const STOP_AFTER: Duration = Duration::from_secs(120);

/// A healthy run self-terminates in ~2s, once the source node has sent its
/// ticks and the closed inputs cascade through to the sink. Anything past this
/// means the dataflow only ended because [`STOP_AFTER`] cut it off. Generous
/// against slow CI while still far below the backstop.
const SELF_TERMINATE_BUDGET: Duration = Duration::from_secs(60);

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

    // The node binaries are pinned to a fixed git rev, so node-API fixes
    // don't reach this example until the rev is bumped.
    let mut run = RunCommand::new(dataflow);
    run.stop_after = Some(STOP_AFTER);

    let started = Instant::now();
    run.execute()?;
    let elapsed = started.elapsed();

    // `execute()` returns `Ok` whether the dataflow ran to completion or was
    // cut off by `stop_after`, and the nodes' own exit codes don't
    // necessarily surface either: a node wedged on a message-format break is
    // killed by SIGTERM at the grace period, which the daemon reports as a
    // clean planned stop on Unix. That combination let #2366 sail through
    // this example on Linux and macOS for nine days while it delivered zero
    // messages end-to-end (#2742) — only Windows went red, and only because
    // it has no SIGTERM and the hard kill surfaced as ExitCode(1).
    //
    // Elapsed time is the one signal that survives all of that, so treat "the
    // dataflow only stopped because the backstop fired" as the failure it is.
    if elapsed >= SELF_TERMINATE_BUDGET {
        bail!(
            "dataflow did not finish on its own within {SELF_TERMINATE_BUDGET:?} \
             (took {elapsed:?}, backstop is {STOP_AFTER:?}).\n\nA healthy run \
             self-terminates in ~2s. Running to the backstop means the nodes \
             wedged rather than completed — most likely a message-format break \
             between the pinned `rev:` and this workspace. See the comment in \
             dataflow.yml."
        );
    }

    Ok(())
}
