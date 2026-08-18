use dora_cli::{Executable, RunCommand};
use eyre::bail;
use std::{env::consts::EXE_SUFFIX, path::Path, time::Duration};

/// Nodes to compile: (source file stem, output binary name).
const NODES: &[(&str, &str)] = &[
    ("service-client", "service_client"),
    ("service-server", "service_server"),
    ("action-client", "action_client"),
    ("action-server", "action_server"),
];

fn main() -> eyre::Result<()> {
    if cfg!(windows) {
        tracing::error!(
            "The c++ example does not work on Windows currently because of a linker error"
        );
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

    // Bound the run so a wedged node fails fast via the daemon's stop
    // escalation instead of hanging until the CI step timeout (#2152).
    // A healthy run self-terminates well before this.
    let mut run = RunCommand::new("dataflow.yml".to_string());
    run.stop_after = Some(Duration::from_secs(60));
    run.execute()?;

    Ok(())
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
    #[cfg(target_os = "linux")]
    {
        clang.arg("-l").arg("m");
        clang.arg("-l").arg("rt");
        clang.arg("-l").arg("dl");
        clang.arg("-l").arg("z");
        clang.arg("-pthread");
    }
    #[cfg(target_os = "macos")]
    {
        clang.arg("-framework").arg("CoreServices");
        clang.arg("-framework").arg("Security");
        clang.arg("-l").arg("System");
        clang.arg("-l").arg("resolv");
        clang.arg("-l").arg("pthread");
        clang.arg("-l").arg("c");
        clang.arg("-l").arg("m");
    }
    for arg in args {
        clang.arg(arg);
    }
    clang.arg("-L").arg(root.join("target").join("debug"));
    clang
        .arg("--output")
        .arg(Path::new("../build").join(format!("{out_name}{EXE_SUFFIX}")));
    // The sources live in `nodes/`; run clang there so the `../build`
    // output path resolves inside this example's directory.
    if let Some(parent) = paths[0].parent() {
        clang.current_dir(parent);
    }

    if !clang.status()?.success() {
        bail!("failed to compile c++ node {out_name}");
    };
    Ok(())
}
