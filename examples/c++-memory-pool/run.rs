use dora_cli::{Executable, RunCommand};
use eyre::bail;
use std::{env::consts::EXE_SUFFIX, path::Path, time::Duration};

/// Nodes to compile: (source file stem, output binary name).
const NODES: &[(&str, &str)] = &[
    ("pool-sender", "pool_sender"),
    ("pool-receiver", "pool_receiver"),
];

/// Where a POSIX shared-memory segment shows up on Linux, and the prefix every
/// pool segment carries.
const SHM_DIR: &str = "/dev/shm";
const SEGMENT_PREFIX: &str = "dora_pool_";

fn main() -> eyre::Result<()> {
    // The transport is a POSIX shared-memory segment reached by name, which
    // is a Linux facility here: `/dev/shm` is where the daemon's unlink and
    // the leak check below both look.
    if !cfg!(target_os = "linux") {
        tracing::error!("the memory-pool example is Linux-only (the pool lives in /dev/shm)");
        return Ok(());
    }

    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let target = root.join("target");
    std::env::set_current_dir(root.join(file!()).parent().unwrap())
        .map_err(|e| eyre::eyre!("failed to set working dir: {e}"))?;

    std::fs::create_dir_all("build")?;

    build_package("dora-node-api-cxx")?;
    let node_cxxbridge = target
        .join("cxxbridge")
        .join("dora-node-api-cxx")
        .join("install");
    let bridge_source = dunce::canonicalize(node_cxxbridge.join("dora-node-api.cc"))?;
    let include_dir = node_cxxbridge
        .as_os_str()
        .to_str()
        .ok_or_else(|| eyre::eyre!("cxxbridge install path is not valid UTF-8"))?;

    for (source_stem, out_name) in NODES {
        build_cxx_node(
            root,
            &[
                &dunce::canonicalize(Path::new("nodes").join(format!("{source_stem}.cc")))?,
                &bridge_source,
            ],
            out_name,
            &["-I", include_dir, "-l", "dora_node_api_cxx"],
        )?;
    }

    // A leftover segment would make the check below fail for a reason that has
    // nothing to do with this run, so say so before the run rather than after.
    let before = pool_segments()?;
    if !before.is_empty() {
        bail!("memory-pool segments were already in {SHM_DIR} before the run: {before:?}");
    }

    // Bound the run so a wedged node fails fast via the daemon's stop
    // escalation instead of hanging until the CI step timeout (#2152).
    // A healthy run self-terminates well before this.
    let mut run = RunCommand::new("dataflow.yml".to_string());
    run.stop_after = Some(Duration::from_secs(60));
    run.execute()?;

    // Each pool is registered with the daemon, which is the only thing that
    // unlinks it. A segment still here is one no node handed back.
    let leaked = pool_segments()?;
    if !leaked.is_empty() {
        bail!("memory-pool segments leaked in {SHM_DIR}: {leaked:?}");
    }

    Ok(())
}

fn pool_segments() -> eyre::Result<Vec<String>> {
    let mut names: Vec<_> = std::fs::read_dir(SHM_DIR)?
        .flatten()
        .map(|entry| entry.file_name().to_string_lossy().into_owned())
        .filter(|name| name.starts_with(SEGMENT_PREFIX))
        .collect();
    names.sort();
    Ok(names)
}

fn build_package(package: &str) -> eyre::Result<()> {
    let cargo = std::env::var("CARGO").unwrap();
    let mut cmd = std::process::Command::new(&cargo);
    cmd.arg("build");
    cmd.arg("--package").arg(package);
    if !cmd.status()?.success() {
        bail!("failed to build {package}");
    };
    Ok(())
}

fn build_cxx_node(root: &Path, paths: &[&Path], out_name: &str, args: &[&str]) -> eyre::Result<()> {
    let mut clang = std::process::Command::new("clang++");
    clang.args(paths);
    clang.arg("-std=c++20");
    for arg in args {
        clang.arg(arg);
    }
    clang.arg("-L").arg(root.join("target").join("debug"));
    // System libraries after the bridge archive, not before: GNU ld resolves
    // an archive's undefined symbols only against what follows it, and on
    // glibc < 2.34 `dlsym`/`shm_open` really are in libdl/librt rather than
    // folded into libc.
    clang.arg("-l").arg("m");
    clang.arg("-l").arg("rt");
    clang.arg("-l").arg("dl");
    clang.arg("-l").arg("z");
    clang.arg("-pthread");
    clang
        .arg("--output")
        .arg(Path::new("../build").join(format!("{out_name}{EXE_SUFFIX}")));
    // The sources live in `nodes/`; run clang there so the `../build` output
    // path and the shared `protocol.h` both resolve.
    if let Some(parent) = paths[0].parent() {
        clang.current_dir(parent);
    }

    // Spawning is reported separately from a non-zero exit: a bare
    // "No such file or directory" here means clang++ itself is missing, which
    // reads as a missing source file otherwise.
    let status = clang.status().map_err(|e| {
        eyre::eyre!("failed to run clang++ (is a C++20-capable clang++ on PATH?): {e}")
    })?;
    if !status.success() {
        bail!("failed to compile c++ node {out_name}");
    };
    Ok(())
}
