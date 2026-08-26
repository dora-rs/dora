use eyre::Context;
use pyo3::prelude::*;

/// Start a runtime for Operators
///
/// :rtype: None
#[pyfunction]
pub fn start_runtime() -> eyre::Result<()> {
    dora_runtime_python::main().wrap_err("Dora Runtime raised an error.")
}

/// Build a Dataflow, exactly the same way as `dora build` command line tool.
///
///
/// :type dataflow_path: str
/// :type uv: bool, optional
/// :type coordinator_addr: str, optional
/// :type coordinator_port: int, optional
/// :type force_local: bool, optional
/// :rtype: None
#[pyfunction]
#[pyo3(signature = (dataflow_path, uv=None, coordinator_addr=None, coordinator_port=None, force_local=false))]
pub fn build(
    dataflow_path: String,
    uv: Option<bool>,
    coordinator_addr: Option<String>,
    coordinator_port: Option<u16>,
    force_local: bool,
) -> eyre::Result<()> {
    dora_cli::build(dora_cli::BuildConfig::from_str_args(
        dataflow_path,
        uv,
        coordinator_addr,
        coordinator_port,
        force_local,
    )?)
}

/// Run a Dataflow, exactly the same way as `dora run` command line tool.
///
/// :type dataflow_path: str
/// :type uv: bool, optional
/// :type stop_after: float, optional
/// :rtype: None
#[pyfunction]
#[pyo3(signature = (dataflow_path, uv=None, stop_after=None))]
pub fn run(dataflow_path: String, uv: Option<bool>, stop_after: Option<f64>) -> eyre::Result<()> {
    use dora_cli::Executable;

    let stop_after_duration = stop_after
        .map(std::time::Duration::try_from_secs_f64)
        .transpose()
        .map_err(|err| eyre::eyre!("invalid stop_after of {stop_after:?} seconds: {err}"))?;
    let mut run = dora_cli::RunCommand::new(dataflow_path);
    if let Some(uv) = uv {
        run.uv = uv;
    }
    if let Some(duration) = stop_after_duration {
        run.stop_after = Some(duration);
    }
    run.execute()
}

/// Entry point for the `dora` console script shipped by the `dora-rs-cli` wheel.
///
/// `pyproject.toml` wires this up as `scripts = { "dora" = "dora_cli:py_main" }`.
///
/// :rtype: None
#[pyfunction]
fn py_main(py: Python<'_>) -> PyResult<()> {
    // Read `sys.argv`, not `std::env::args_os()`. The process argv depends on
    // how the console script is launched -- on Unix the shebang puts the
    // interpreter in argv[0] and the script in argv[1], while a Windows
    // `dora.exe` wrapper has no interpreter entry at all -- so any fixed
    // `skip(n)` is wrong on one platform or the other. `sys.argv` is already
    // normalized to `[script_path, ..args]`, which is exactly clap's shape.
    //
    // Extract straight to `OsString`: a path argument that is not valid UTF-8
    // reaches Python surrogate-escaped, and going through `String` would reject
    // it here rather than passing it on to the filesystem.
    let argv: Vec<std::ffi::OsString> = py.import("sys")?.getattr("argv")?.extract()?;

    // Same entry point `binaries/cli/src/main.rs` uses, so argument handling and
    // exit codes cannot drift between the wheel and the standalone binary.
    // Runs detached: the command can take minutes and hosts the in-process
    // daemon, and holding the GIL that long would block the interpreter.
    py.detach(|| dora_cli::lib_main_from_argv(argv));
    Ok(())
}

#[pymodule(name = "dora_cli")]
fn dora_cli_python(_py: Python, m: Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(py_main, &m)?)?;
    m.add_function(wrap_pyfunction!(start_runtime, &m)?)?;
    m.add_function(wrap_pyfunction!(run, &m)?)?;
    m.add_function(wrap_pyfunction!(build, &m)?)?;
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::{Path, PathBuf};

    /// Read at compile time, so if the wheel manifest ever moves out of this
    /// crate again the build breaks here rather than silently shipping a wheel
    /// built from a crate with no `#[pymodule]` in it.
    const PYPROJECT: &str = include_str!("../pyproject.toml");

    fn repo_root() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .ancestors()
            .nth(3)
            .expect("crate is three levels below the repo root")
            .to_path_buf()
    }

    /// `[project.scripts]` as `(script_name, module, attribute)`. Empty when the
    /// wheel declares none -- `dora-rs` is a library wheel with no `dora`-style
    /// command, and that is not a defect.
    fn console_scripts(pyproject: &str) -> Vec<(String, String, String)> {
        let manifest: toml::Table = pyproject.parse().expect("pyproject.toml is not valid TOML");
        let Some(scripts) = manifest["project"]
            .get("scripts")
            .and_then(toml::Value::as_table)
        else {
            return Vec::new();
        };
        scripts
            .iter()
            .map(|(name, target)| {
                let target = target
                    .as_str()
                    .expect("console script target is not a string");
                let (module, attr) = target.split_once(':').unwrap_or_else(|| {
                    panic!("console script `{name}` target `{target}` is not `module:attr`")
                });
                (name.clone(), module.to_owned(), attr.to_owned())
            })
            .collect()
    }

    /// The `dora` command is a console script pointing into this extension
    /// module. Nothing in a normal `cargo build` fails when that target stops
    /// existing -- the wheel just ships a `dora` that dies with `ImportError`
    /// the first time a user runs it, which is how the entry point stayed
    /// missing for months. Import the real module and check.
    #[test]
    fn console_scripts_resolve_against_the_module() {
        pyo3::append_to_inittab!(dora_cli_python);

        let scripts = console_scripts(PYPROJECT);
        assert!(
            !scripts.is_empty(),
            "pyproject.toml declares no [project.scripts]; the `dora` command is \
             the reason this wheel exists"
        );

        Python::attach(|py| {
            let module = py.import("dora_cli").expect("`dora_cli` failed to import");
            for (script, target_module, attr) in scripts {
                assert_eq!(
                    target_module, "dora_cli",
                    "console script `{script}` targets `{target_module}`, which this crate does not build"
                );
                assert!(
                    module.hasattr(&attr).unwrap(),
                    "console script `{script}` needs `dora_cli:{attr}`, but the module does not export `{attr}`"
                );
            }
        });
    }

    /// The deeper half of the same invariant. The entry point above can only
    /// vouch for *this* crate; `pip-release.yml` decides which crate actually
    /// becomes a wheel, and it previously pointed at one with no `#[pymodule]`
    /// at all -- a manifest and a module sitting in two different crates, with
    /// nothing comparing them. Check every path the release workflow builds.
    #[test]
    fn every_released_wheel_crate_can_provide_its_console_scripts() {
        let root = repo_root();
        let workflow = std::fs::read_to_string(root.join(".github/workflows/pip-release.yml"))
            .expect("pip-release.yml is missing");
        let paths = wheel_paths(&workflow);
        assert!(
            paths.contains(&"apis/python/cli".to_owned()),
            "pip-release.yml no longer builds this crate; it published {paths:?}"
        );

        for path in paths {
            let crate_dir = root.join(&path);
            let pyproject = std::fs::read_to_string(crate_dir.join("pyproject.toml"))
                .unwrap_or_else(|_| {
                    panic!(
                        "pip-release.yml builds a wheel from `{path}`, which has no pyproject.toml"
                    )
                });
            let manifest = std::fs::read_to_string(crate_dir.join("Cargo.toml"))
                .unwrap_or_else(|_| panic!("`{path}` has no Cargo.toml"));
            let lib_name = manifest
                .parse::<toml::Table>()
                .expect("Cargo.toml is not valid TOML")
                .get("lib")
                .and_then(|lib| lib.get("name"))
                .and_then(toml::Value::as_str)
                .unwrap_or_else(|| panic!("`{path}` declares no [lib] name to import as"))
                .to_owned();

            // The check that would have caught the original break: a crate can
            // only answer `import <module>` if it defines the module.
            let sources = std::fs::read_dir(crate_dir.join("src"))
                .unwrap_or_else(|_| panic!("`{path}` has no src/"))
                .filter_map(|entry| std::fs::read_to_string(entry.ok()?.path()).ok())
                .collect::<String>();
            assert!(
                sources.contains("#[pymodule"),
                "pip-release.yml builds a wheel from `{path}`, but that crate defines no \
                 `#[pymodule]` -- the wheel would ship an extension module with no init symbol"
            );

            for (script, module, _) in console_scripts(&pyproject) {
                assert_eq!(
                    module, lib_name,
                    "`{path}` console script `{script}` imports `{module}`, but the crate builds `{lib_name}`"
                );
            }
        }
    }

    /// `matrix.repository[].path` values from the release workflow, deduplicated.
    /// Hand-scanned rather than YAML-parsed so the test needs no serde_yaml: the
    /// build matrix repeats the same two-entry list once per platform job.
    fn wheel_paths(workflow: &str) -> Vec<String> {
        let mut paths: Vec<String> = workflow
            .lines()
            .filter_map(|line| Some(line.trim().strip_prefix("- path:")?.trim().to_owned()))
            .collect();
        paths.sort();
        paths.dedup();
        assert!(
            !paths.is_empty(),
            "found no `- path:` entries in pip-release.yml"
        );
        paths
    }
}
