use std::{
    collections::BTreeMap,
    ffi::OsString,
    fs,
    path::{Path, PathBuf},
    process::Stdio,
    sync::LazyLock,
    time::Duration,
};

use crate::build::{managed_python_bin_dir, managed_python_interpreter};
use dora_message::descriptor::EnvValue;
use eyre::{Context, eyre};
use tokio::{
    io::{AsyncBufRead, AsyncBufReadExt, BufReader},
    process::Command,
};
use tokio_stream::{StreamExt, wrappers::LinesStream};

/// Serializes wheel builds within one process: concurrent env preparations share the
/// per-PID staging dir, so only one `uv build` may use it at a time.
static LOCAL_DORA_WHEEL_BUILD_LOCK: LazyLock<tokio::sync::Mutex<()>> =
    LazyLock::new(|| tokio::sync::Mutex::new(()));

const LOCAL_DORA_RUNTIME_MARKER: &str = ".dora-local-runtime-fingerprint";
const LOCAL_DORA_WHEEL_CACHE_MAX_AGE: Duration = Duration::from_secs(24 * 60 * 60);
const FNV_OFFSET_BASIS: u64 = 0xcbf29ce484222325;
const FNV_PRIME: u64 = 0x00000100000001b3;

#[derive(Debug)]
struct LocalDoraPythonSource {
    package_dir: PathBuf,
    fingerprint: String,
}

pub async fn run_build_command(
    build: &str,
    working_dir: &Path,
    uv: bool,
    python_env_dir: Option<PathBuf>,
    envs: &Option<BTreeMap<String, EnvValue>>,
    stdout_tx: tokio::sync::mpsc::Sender<std::io::Result<String>>,
) -> eyre::Result<()> {
    std::fs::create_dir_all(working_dir).context("failed to create working directory")?;

    let lines = build.lines().collect::<Vec<_>>();
    for build_line in lines {
        let mut split = splitty::split_unquoted_whitespace(build_line).unwrap_quotes(true);

        let program = split
            .next()
            .ok_or_else(|| eyre!("build command is empty"))?;
        let mut cmd = if uv && (program == "pip" || program == "pip3") {
            let mut cmd = Command::new("uv");
            cmd.arg("pip");
            cmd
        } else {
            Command::new(program)
        };
        cmd.args(split);

        // Inject Environment Variables
        if let Some(envs) = envs {
            for (key, value) in envs {
                let value = value.to_string();
                cmd.env(key, value);
            }
        }
        if uv && let Some(python_env_dir) = &python_env_dir {
            apply_managed_python_env(&mut cmd, python_env_dir, envs)
                .context("failed to apply managed Python env")?;
        }

        cmd.current_dir(dunce::simplified(working_dir));

        cmd.stdin(Stdio::null());
        cmd.stdout(Stdio::piped());
        cmd.stderr(Stdio::piped());

        cmd.env("CLICOLOR", "1");
        cmd.env("CLICOLOR_FORCE", "1");

        let mut child = cmd
            .spawn()
            .wrap_err_with(|| format!("failed to spawn `{build}`"))?;

        let child_stdout = BufReader::new(
            child
                .stdout
                .take()
                .ok_or_else(|| eyre!("failed to capture stdout pipe from build command"))?,
        );
        let child_stderr = BufReader::new(
            child
                .stderr
                .take()
                .ok_or_else(|| eyre!("failed to capture stderr pipe from build command"))?,
        );
        let stdout_tx = stdout_tx.clone();

        tokio::spawn(async move {
            forward_build_output(child_stdout, child_stderr, stdout_tx).await;
        });

        let exit_status = child
            .wait()
            .await
            .wrap_err_with(|| format!("failed to run `{build}`"))?;
        if !exit_status.success() {
            return Err(eyre!("build command `{build_line}` returned {exit_status}"));
        }
    }
    Ok(())
}

/// Sets `VIRTUAL_ENV` and prepends the managed env's bin dir to `PATH` on `cmd`.
///
/// Fails closed if the managed interpreter is missing: falling through to the ambient
/// `PATH` could silently pick a different Python than the one Dora prepared for this node.
fn apply_managed_python_env(
    cmd: &mut Command,
    python_env_dir: &Path,
    envs: &Option<BTreeMap<String, EnvValue>>,
) -> eyre::Result<()> {
    let interpreter = managed_python_interpreter(python_env_dir);
    if !interpreter.is_file() {
        return Err(eyre!(
            "managed Python interpreter `{}` is missing; run Dora's Python env preparation first",
            interpreter.display()
        ));
    }
    cmd.env("VIRTUAL_ENV", python_env_dir);
    if let Some(path) = managed_python_path(python_env_dir, envs)? {
        cmd.env("PATH", path);
    }
    Ok(())
}

/// Composes the PATH for a spawned command: managed bin dir first, then user envs, then ambient.
fn managed_python_path(
    python_env_dir: &Path,
    envs: &Option<BTreeMap<String, EnvValue>>,
) -> eyre::Result<Option<OsString>> {
    // Prepend the managed env so `python`/`pip` resolve there before any machine-level entries.
    let mut paths = vec![managed_python_bin_dir(python_env_dir)];
    let base_path = envs
        .as_ref()
        .and_then(|envs| envs.get("PATH"))
        .map(|value| OsString::from(value.to_string()))
        .or_else(|| std::env::var_os("PATH"));

    if let Some(base_path) = base_path {
        paths.extend(std::env::split_paths(&base_path));
    }

    std::env::join_paths(paths)
        .map(Some)
        .wrap_err("failed to compose managed Python PATH")
}

/// Creates the managed Python venv if missing and ensures `dora` is importable inside it.
///
/// Idempotent: returns immediately if both the interpreter exists and `import dora` succeeds.
/// Uses `uv venv --clear` for creation and `uv pip install` for the runtime install,
/// preferring an in-tree workspace package when available.
pub async fn prepare_managed_python_env(
    working_dir: &Path,
    python_env_dir: &Path,
    envs: &Option<BTreeMap<String, EnvValue>>,
    stdout_tx: tokio::sync::mpsc::Sender<std::io::Result<String>>,
) -> eyre::Result<()> {
    std::fs::create_dir_all(working_dir).context("failed to create working directory")?;
    if let Some(parent) = python_env_dir.parent() {
        std::fs::create_dir_all(parent).context("failed to create Python env parent dir")?;
    }
    if !managed_python_interpreter(python_env_dir).is_file() {
        let mut cmd = Command::new("uv");
        cmd.arg("venv");
        cmd.arg("--clear");
        cmd.arg(python_env_dir);

        if let Some(envs) = envs {
            for (key, value) in envs {
                cmd.env(key, value.to_string());
            }
        }

        cmd.current_dir(dunce::simplified(working_dir));
        cmd.stdin(Stdio::null());
        cmd.stdout(Stdio::piped());
        cmd.stderr(Stdio::piped());
        cmd.env("CLICOLOR", "1");
        cmd.env("CLICOLOR_FORCE", "1");

        let mut child = cmd
            .spawn()
            .wrap_err_with(|| format!("failed to spawn `uv venv {}`", python_env_dir.display()))?;

        let child_stdout = BufReader::new(
            child
                .stdout
                .take()
                .ok_or_else(|| eyre!("failed to take stdout pipe from uv venv child"))?,
        );
        let child_stderr = BufReader::new(
            child
                .stderr
                .take()
                .ok_or_else(|| eyre!("failed to take stderr pipe from uv venv child"))?,
        );
        let stdout_tx_clone = stdout_tx.clone();

        tokio::spawn(async move {
            forward_build_output(child_stdout, child_stderr, stdout_tx_clone).await;
        });

        let exit_status = child
            .wait()
            .await
            .wrap_err_with(|| format!("failed to run `uv venv {}`", python_env_dir.display()))?;
        if !exit_status.success() {
            return Err(eyre!(
                "managed Python env preparation `{}` returned {exit_status}",
                python_env_dir.display()
            ));
        }
    }

    ensure_managed_python_runtime(working_dir, python_env_dir, envs, stdout_tx).await
}

/// Installs `dora` into the managed env if `import dora` doesn't succeed there yet.
async fn ensure_managed_python_runtime(
    working_dir: &Path,
    python_env_dir: &Path,
    envs: &Option<BTreeMap<String, EnvValue>>,
    stdout_tx: tokio::sync::mpsc::Sender<std::io::Result<String>>,
) -> eyre::Result<()> {
    let local_source = local_dora_python_source(working_dir)?;
    let can_import_dora = managed_python_can_import_dora(working_dir, python_env_dir, envs).await?;
    if local_source
        .as_ref()
        .is_some_and(|source| local_runtime_marker_matches(python_env_dir, &source.fingerprint))
        && can_import_dora
    {
        return Ok(());
    }
    if local_source.is_none() && can_import_dora {
        return Ok(());
    }

    let mut cmd = Command::new("uv");
    cmd.arg("pip");
    cmd.arg("install");
    let install_args =
        dora_runtime_install_args(local_source.as_ref(), envs, stdout_tx.clone()).await?;
    cmd.args(install_args);

    if let Some(envs) = envs {
        for (key, value) in envs {
            cmd.env(key, value.to_string());
        }
    }
    apply_managed_python_env(&mut cmd, python_env_dir, envs)
        .context("failed to target managed env for Dora runtime install")?;
    cmd.current_dir(dunce::simplified(working_dir));
    cmd.stdin(Stdio::null());
    cmd.stdout(Stdio::piped());
    cmd.stderr(Stdio::piped());
    cmd.env("CLICOLOR", "1");
    cmd.env("CLICOLOR_FORCE", "1");

    let mut child = cmd.spawn().wrap_err_with(|| {
        format!(
            "failed to spawn Dora runtime install into `{}`",
            python_env_dir.display()
        )
    })?;

    let child_stdout = BufReader::new(
        child
            .stdout
            .take()
            .ok_or_else(|| eyre!("failed to capture stdout pipe from Dora runtime install"))?,
    );
    let child_stderr = BufReader::new(
        child
            .stderr
            .take()
            .ok_or_else(|| eyre!("failed to capture stderr pipe from Dora runtime install"))?,
    );

    tokio::spawn(async move {
        forward_build_output(child_stdout, child_stderr, stdout_tx).await;
    });

    let exit_status = child.wait().await.wrap_err_with(|| {
        format!(
            "failed to install Dora runtime into `{}`",
            python_env_dir.display()
        )
    })?;
    if !exit_status.success() {
        let mut err = eyre!(
            "managed Python runtime installation `{}` returned {exit_status}",
            python_env_dir.display()
        );
        if local_source.is_none() {
            err = err.wrap_err(format!(
                "failed to install dora-rs=={} from PyPI. If you are running a \
                 pre-release/dev build of dora, that version is not published — \
                 either run the dataflow from inside a dora source checkout, or \
                 set DORA_PYTHON_RUNTIME_SOURCE=<dora-repo>/apis/python/node",
                env!("CARGO_PKG_VERSION")
            ));
        }
        return Err(err);
    }

    if let Some(source) = local_source {
        write_local_runtime_marker(python_env_dir, &source.fingerprint)?;
    }

    Ok(())
}

/// Returns true if `python -c 'import dora'` succeeds inside the managed env.
async fn managed_python_can_import_dora(
    working_dir: &Path,
    python_env_dir: &Path,
    envs: &Option<BTreeMap<String, EnvValue>>,
) -> eyre::Result<bool> {
    let interpreter = managed_python_interpreter(python_env_dir);
    let mut cmd = Command::new(&interpreter);
    apply_managed_python_env(&mut cmd, python_env_dir, envs)
        .context("failed to target managed env for Dora runtime probe")?;
    cmd.arg("-c");
    cmd.arg("import dora");
    cmd.current_dir(dunce::simplified(working_dir));
    cmd.stdin(Stdio::null());
    cmd.stdout(Stdio::null());
    cmd.stderr(Stdio::null());

    let exit_status = cmd.status().await.wrap_err_with(|| {
        format!(
            "failed to probe Dora runtime in `{}`",
            python_env_dir.display()
        )
    })?;
    Ok(exit_status.success())
}

/// Walk up from `working_dir` looking for `apis/python/node/pyproject.toml`; returns the
/// directory containing it if found (so in-tree dev workflows install the local Python
/// package instead of pulling from PyPI).
fn local_dora_python_package_dir(working_dir: &Path) -> Option<PathBuf> {
    // Explicit override for dataflows outside a dora source checkout running
    // against a pre-release CLI whose dora-rs version is not on PyPI.
    if let Ok(source) = std::env::var("DORA_PYTHON_RUNTIME_SOURCE") {
        let package_dir = PathBuf::from(source);
        if package_dir.join("pyproject.toml").is_file() {
            return Some(package_dir);
        }
        tracing::warn!(
            "DORA_PYTHON_RUNTIME_SOURCE `{}` has no pyproject.toml — ignoring override",
            package_dir.display()
        );
    }
    working_dir.ancestors().find_map(|dir| {
        let package_dir = dir.join("apis").join("python").join("node");
        package_dir
            .join("pyproject.toml")
            .is_file()
            .then_some(package_dir)
    })
}

fn local_dora_python_source(working_dir: &Path) -> eyre::Result<Option<LocalDoraPythonSource>> {
    local_dora_python_package_dir(working_dir)
        .map(|package_dir| {
            let package_dir = dunce::canonicalize(&package_dir).wrap_err_with(|| {
                format!(
                    "failed to canonicalize local Dora Python package `{}`",
                    package_dir.display()
                )
            })?;
            let fingerprint = local_dora_python_source_fingerprint(&package_dir)?;
            Ok(LocalDoraPythonSource {
                package_dir,
                fingerprint,
            })
        })
        .transpose()
}

/// Returns the package argument for installing `dora-rs` into a managed Python env.
async fn dora_runtime_install_args(
    local_source: Option<&LocalDoraPythonSource>,
    envs: &Option<BTreeMap<String, EnvValue>>,
    stdout_tx: tokio::sync::mpsc::Sender<std::io::Result<String>>,
) -> eyre::Result<Vec<OsString>> {
    match local_source {
        Some(source) => Ok(vec![
            OsString::from("--reinstall-package"),
            OsString::from("dora-rs"),
            ensure_local_dora_python_wheel(source, envs, stdout_tx)
                .await?
                .into_os_string(),
        ]),
        None => Ok(vec![OsString::from(format!(
            "dora-rs=={}",
            env!("CARGO_PKG_VERSION")
        ))]),
    }
}

/// Builds the in-tree Python API at most once per source fingerprint and returns its wheel path.
///
/// Installing the local source tree directly asks maturin to build once per managed
/// node env. The wheel is built into a per-process staging dir and then promoted to a
/// fingerprint-and-platform-keyed shared dir under `target/dora-python-wheels/`, so
/// subsequent Dora processes (separate `dora build`/`dora run` invocations, each smoke
/// test) reuse one build instead of recompiling per process (#2143).
///
/// The fingerprint does not cover workspace dependency crates outside
/// `apis/python/node` (same scope the env reinstall marker already accepts), so shared
/// wheels are age-evicted after [`LOCAL_DORA_WHEEL_CACHE_MAX_AGE`] to bound that drift.
/// The shared dir is the only cache layer — deliberately no in-memory memoization, so
/// the eviction also applies inside long-lived processes like the daemon, which serves
/// many builds without restarting.
async fn ensure_local_dora_python_wheel(
    source: &LocalDoraPythonSource,
    envs: &Option<BTreeMap<String, EnvValue>>,
    stdout_tx: tokio::sync::mpsc::Sender<std::io::Result<String>>,
) -> eyre::Result<PathBuf> {
    let _build_guard = LOCAL_DORA_WHEEL_BUILD_LOCK.lock().await;

    let wheel_root = local_dora_python_wheel_root(&source.package_dir)?;
    cleanup_stale_local_dora_python_wheel_dirs(&wheel_root, LOCAL_DORA_WHEEL_CACHE_MAX_AGE);

    let shared_dir = local_dora_python_shared_wheel_dir(&wheel_root, &source.fingerprint);
    if let Ok(wheel) = find_local_dora_python_wheel(&shared_dir) {
        return Ok(wheel);
    }

    let wheel_dir = wheel_root.join(format!("build-{}", std::process::id()));
    std::fs::create_dir_all(&wheel_dir).wrap_err_with(|| {
        format!(
            "failed to create local Dora Python wheel staging dir `{}`",
            wheel_dir.display()
        )
    })?;

    let mut cmd = Command::new("uv");
    cmd.arg("build");
    cmd.arg("--wheel");
    cmd.arg("--clear");
    cmd.arg("--no-create-gitignore");
    cmd.arg("--out-dir");
    cmd.arg(&wheel_dir);
    cmd.arg(&source.package_dir);

    if let Some(envs) = envs {
        for (key, value) in envs {
            cmd.env(key, value.to_string());
        }
    }
    cmd.current_dir(dunce::simplified(&source.package_dir));
    cmd.stdin(Stdio::null());
    cmd.stdout(Stdio::piped());
    cmd.stderr(Stdio::piped());
    cmd.env("CLICOLOR", "1");
    cmd.env("CLICOLOR_FORCE", "1");

    let mut child = cmd.spawn().wrap_err_with(|| {
        format!(
            "failed to spawn local Dora Python wheel build for `{}`",
            source.package_dir.display()
        )
    })?;

    let child_stdout = BufReader::new(
        child
            .stdout
            .take()
            .ok_or_else(|| eyre!("failed to capture stdout pipe from local Dora wheel build"))?,
    );
    let child_stderr = BufReader::new(
        child
            .stderr
            .take()
            .ok_or_else(|| eyre!("failed to capture stderr pipe from local Dora wheel build"))?,
    );

    tokio::spawn(async move {
        forward_build_output(child_stdout, child_stderr, stdout_tx).await;
    });

    let exit_status = child.wait().await.wrap_err_with(|| {
        format!(
            "failed to build local Dora Python wheel from `{}`",
            source.package_dir.display()
        )
    })?;
    if !exit_status.success() {
        return Err(eyre!(
            "local Dora Python wheel build `{}` returned {exit_status}",
            source.package_dir.display()
        ));
    }

    promote_local_dora_python_wheel(&wheel_dir, &shared_dir)
}

fn local_dora_python_wheel_root(package_dir: &Path) -> eyre::Result<PathBuf> {
    Ok(local_dora_workspace_root(package_dir)?
        .join("target")
        .join("dora-python-wheels"))
}

/// The key includes OS and architecture so checkouts visible to several platforms
/// (e.g. WSL + native Windows, NFS-shared workspaces) don't serve each other
/// incompatible wheels.
fn local_dora_python_shared_wheel_dir(wheel_root: &Path, fingerprint: &str) -> PathBuf {
    wheel_root.join(format!(
        "fp-{fingerprint}-{}-{}",
        std::env::consts::OS,
        std::env::consts::ARCH
    ))
}

/// Moves a freshly built wheel from the per-process staging dir into the shared
/// fingerprint-keyed dir. If another process promoted the same fingerprint first,
/// its wheel wins and the staging dir is discarded. A wheel-less shared dir (e.g.
/// left by an interrupted cleanup) is replaced so the cache self-heals.
fn promote_local_dora_python_wheel(staging_dir: &Path, shared_dir: &Path) -> eyre::Result<PathBuf> {
    let staged_wheel = find_local_dora_python_wheel(staging_dir)?;
    let wheel_name = staged_wheel
        .file_name()
        .map(ToOwned::to_owned)
        .ok_or_else(|| eyre!("staged wheel `{}` has no file name", staged_wheel.display()))?;

    if let Err(rename_error) = fs::rename(staging_dir, shared_dir) {
        if let Ok(wheel) = find_local_dora_python_wheel(shared_dir) {
            let _ = fs::remove_dir_all(staging_dir);
            return Ok(wheel);
        }
        let _ = fs::remove_dir_all(shared_dir);
        if fs::rename(staging_dir, shared_dir).is_err() {
            if let Ok(wheel) = find_local_dora_python_wheel(shared_dir) {
                let _ = fs::remove_dir_all(staging_dir);
                return Ok(wheel);
            }
            return Err(rename_error).wrap_err_with(|| {
                format!(
                    "failed to promote local Dora Python wheel from `{}` to `{}`",
                    staging_dir.display(),
                    shared_dir.display()
                )
            });
        }
    }
    Ok(shared_dir.join(wheel_name))
}

fn find_local_dora_python_wheel(wheel_dir: &Path) -> eyre::Result<PathBuf> {
    let mut wheels = std::fs::read_dir(wheel_dir)
        .wrap_err_with(|| format!("failed to read wheel cache `{}`", wheel_dir.display()))?
        .filter_map(|entry| entry.ok())
        .map(|entry| entry.path())
        .filter(|path| {
            path.extension().is_some_and(|extension| extension == "whl")
                && path
                    .file_name()
                    .is_some_and(|name| name.to_string_lossy().starts_with("dora_rs-"))
        })
        .collect::<Vec<_>>();
    wheels.sort();
    wheels.into_iter().next_back().ok_or_else(|| {
        eyre!(
            "local Dora Python wheel build did not create a dora-rs wheel in `{}`",
            wheel_dir.display()
        )
    })
}

/// Age-evicts Dora-managed wheel dirs under `wheel_root`.
///
/// Evicting stale `fp-` dirs uniformly (including the current fingerprint's, which is
/// rebuilt right afterwards) bounds how long a cached wheel can drift from sources the
/// fingerprint doesn't cover. In-flight `build-` staging dirs are protected by their
/// fresh mtime, not by name: `uv build --clear` rewrites the dir when a build starts.
fn cleanup_stale_local_dora_python_wheel_dirs(wheel_root: &Path, max_age: Duration) {
    let Ok(entries) = fs::read_dir(wheel_root) else {
        return;
    };

    for entry in entries.filter_map(Result::ok) {
        let path = entry.path();
        if !entry.file_type().is_ok_and(|file_type| file_type.is_dir()) {
            continue;
        }

        let Some(name) = path.file_name().and_then(|name| name.to_str()) else {
            continue;
        };
        // `process-` covers wheel dirs written by older Dora versions.
        let managed =
            name.starts_with("fp-") || name.starts_with("build-") || name.starts_with("process-");
        if !managed {
            continue;
        }

        let stale = path
            .metadata()
            .and_then(|metadata| metadata.modified())
            .and_then(|modified| modified.elapsed().map_err(std::io::Error::other))
            .is_ok_and(|age| age >= max_age);
        if stale {
            let _ = fs::remove_dir_all(path);
        }
    }
}

fn local_runtime_marker_path(python_env_dir: &Path) -> PathBuf {
    python_env_dir.join(LOCAL_DORA_RUNTIME_MARKER)
}

fn local_runtime_marker_matches(python_env_dir: &Path, fingerprint: &str) -> bool {
    std::fs::read_to_string(local_runtime_marker_path(python_env_dir))
        .is_ok_and(|existing| existing.trim() == fingerprint)
}

fn write_local_runtime_marker(python_env_dir: &Path, fingerprint: &str) -> eyre::Result<()> {
    std::fs::write(local_runtime_marker_path(python_env_dir), fingerprint).wrap_err_with(|| {
        format!(
            "failed to write local Dora runtime marker in `{}`",
            python_env_dir.display()
        )
    })
}

fn local_dora_workspace_root(package_dir: &Path) -> eyre::Result<PathBuf> {
    package_dir
        .parent()
        .and_then(|path| path.parent())
        .and_then(|path| path.parent())
        .map(Path::to_path_buf)
        .ok_or_else(|| {
            eyre!(
                "local Dora Python package `{}` is not under apis/python/node",
                package_dir.display()
            )
        })
}

fn local_dora_python_source_fingerprint(package_dir: &Path) -> eyre::Result<String> {
    let workspace_root = local_dora_workspace_root(package_dir)?;
    let mut files = Vec::new();
    for file_name in ["Cargo.lock", "Cargo.toml"] {
        let path = workspace_root.join(file_name);
        if path.is_file() {
            files.push(path);
        }
    }
    collect_fingerprint_files(package_dir, &mut files)?;
    files.sort();

    let mut hash = FNV_OFFSET_BASIS;
    for path in files {
        let relative = path.strip_prefix(&workspace_root).unwrap_or(&path);
        hash_fingerprint_bytes(&mut hash, relative.to_string_lossy().as_bytes());
        hash_fingerprint_bytes(&mut hash, b"\0");
        let contents = fs::read(&path)
            .wrap_err_with(|| format!("failed to read `{}` for fingerprint", path.display()))?;
        hash_fingerprint_bytes(&mut hash, &contents);
        hash_fingerprint_bytes(&mut hash, b"\0");
    }

    Ok(format!("{hash:016x}"))
}

fn collect_fingerprint_files(dir: &Path, files: &mut Vec<PathBuf>) -> eyre::Result<()> {
    let mut entries = fs::read_dir(dir)
        .wrap_err_with(|| format!("failed to read `{}` for fingerprint", dir.display()))?
        .collect::<Result<Vec<_>, _>>()
        .wrap_err_with(|| format!("failed to read entry under `{}`", dir.display()))?;
    entries.sort_by_key(|entry| entry.path());

    for entry in entries {
        let path = entry.path();
        let file_type = entry
            .file_type()
            .wrap_err_with(|| format!("failed to inspect `{}` for fingerprint", path.display()))?;
        if file_type.is_dir() {
            if should_skip_fingerprint_dir(&path) {
                continue;
            }
            collect_fingerprint_files(&path, files)?;
        } else if file_type.is_file() && is_fingerprint_file(&path) {
            files.push(path);
        }
    }

    Ok(())
}

fn should_skip_fingerprint_dir(path: &Path) -> bool {
    path.file_name()
        .and_then(|name| name.to_str())
        .is_some_and(|name| {
            matches!(
                name,
                ".direnv"
                    | ".mypy_cache"
                    | ".pytest_cache"
                    | ".ruff_cache"
                    | ".tox"
                    | ".venv"
                    | "__pycache__"
                    | "out"
                    | "target"
            )
        })
}

fn is_fingerprint_file(path: &Path) -> bool {
    if path
        .file_name()
        .and_then(|name| name.to_str())
        .is_some_and(|name| matches!(name, "Cargo.toml" | "build.rs" | "pyproject.toml"))
    {
        return true;
    }

    path.extension()
        .and_then(|extension| extension.to_str())
        .is_some_and(|extension| matches!(extension, "py" | "rs" | "toml"))
}

fn hash_fingerprint_bytes(hash: &mut u64, bytes: &[u8]) {
    for byte in bytes {
        *hash ^= u64::from(*byte);
        *hash = hash.wrapping_mul(FNV_PRIME);
    }
}

/// Forwards stdout and stderr to the build logger until both streams close.
async fn forward_build_output<R1, R2>(
    child_stdout: R1,
    child_stderr: R2,
    stdout_tx: tokio::sync::mpsc::Sender<std::io::Result<String>>,
) where
    R1: AsyncBufRead + Unpin,
    R2: AsyncBufRead + Unpin,
{
    let mut merged =
        LinesStream::new(child_stdout.lines()).merge(LinesStream::new(child_stderr.lines()));

    while let Some(line) = merged.next().await {
        if stdout_tx.send(line).await.is_err() {
            break;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{
        cleanup_stale_local_dora_python_wheel_dirs, dora_runtime_install_args,
        find_local_dora_python_wheel, forward_build_output, local_dora_python_package_dir,
        local_dora_python_shared_wheel_dir, local_dora_python_source_fingerprint,
        local_dora_python_wheel_root, local_runtime_marker_matches,
        promote_local_dora_python_wheel, write_local_runtime_marker,
    };
    use std::{ffi::OsString, fs, time::Duration};
    use tokio::io::{AsyncWriteExt, BufReader};

    #[test]
    fn local_runtime_install_discovers_workspace_package() {
        let temp = tempfile::tempdir().unwrap();
        let package_dir = temp.path().join("apis").join("python").join("node");
        fs::create_dir_all(&package_dir).unwrap();
        fs::write(
            package_dir.join("pyproject.toml"),
            "[project]\nname = \"dora-rs\"\n",
        )
        .unwrap();

        let working_dir = temp.path().join("examples").join("python-dataflow");
        fs::create_dir_all(&working_dir).unwrap();

        assert_eq!(
            local_dora_python_package_dir(&working_dir),
            Some(package_dir)
        );
    }

    #[tokio::test]
    async fn runtime_install_falls_back_to_versioned_pypi_requirement() {
        let (stdout_tx, _stdout_rx) = tokio::sync::mpsc::channel(1);

        assert_eq!(
            dora_runtime_install_args(None, &None, stdout_tx)
                .await
                .unwrap(),
            vec![OsString::from(format!(
                "dora-rs=={}",
                env!("CARGO_PKG_VERSION")
            ))]
        );
    }

    #[test]
    fn local_runtime_wheel_cache_is_keyed_by_fingerprint_and_platform() {
        let temp = tempfile::tempdir().unwrap();
        let package_dir = temp.path().join("apis").join("python").join("node");
        fs::create_dir_all(&package_dir).unwrap();

        let wheel_root = local_dora_python_wheel_root(&package_dir).unwrap();
        assert_eq!(
            wheel_root,
            temp.path().join("target").join("dora-python-wheels")
        );
        assert_eq!(
            local_dora_python_shared_wheel_dir(&wheel_root, "abc123"),
            wheel_root.join(format!(
                "fp-abc123-{}-{}",
                std::env::consts::OS,
                std::env::consts::ARCH
            ))
        );
    }

    #[test]
    fn promotes_staged_wheel_to_shared_dir() {
        let temp = tempfile::tempdir().unwrap();
        let staging_dir = temp.path().join("build-1");
        let shared_dir = temp.path().join("fp-abc");
        let wheel_name = "dora_rs-0.4.0-cp37-abi3-manylinux_2_34_x86_64.whl";
        fs::create_dir_all(&staging_dir).unwrap();
        fs::write(staging_dir.join(wheel_name), "").unwrap();

        let wheel = promote_local_dora_python_wheel(&staging_dir, &shared_dir).unwrap();

        assert_eq!(wheel, shared_dir.join(wheel_name));
        assert!(!staging_dir.exists());
    }

    #[test]
    fn promote_uses_existing_shared_wheel_when_losing_the_race() {
        let temp = tempfile::tempdir().unwrap();
        let staging_dir = temp.path().join("build-1");
        let shared_dir = temp.path().join("fp-abc");
        let winner_wheel = "dora_rs-0.4.0-cp37-abi3-manylinux_2_34_aarch64.whl";
        fs::create_dir_all(&staging_dir).unwrap();
        fs::write(
            staging_dir.join("dora_rs-0.4.0-cp37-abi3-manylinux_2_34_x86_64.whl"),
            "",
        )
        .unwrap();
        fs::create_dir_all(&shared_dir).unwrap();
        fs::write(shared_dir.join(winner_wheel), "").unwrap();

        let wheel = promote_local_dora_python_wheel(&staging_dir, &shared_dir).unwrap();

        assert_eq!(wheel, shared_dir.join(winner_wheel));
        assert!(!staging_dir.exists());
    }

    #[test]
    fn promote_fails_when_staging_has_no_wheel() {
        let temp = tempfile::tempdir().unwrap();
        let staging_dir = temp.path().join("build-1");
        let shared_dir = temp.path().join("fp-abc");
        fs::create_dir_all(&staging_dir).unwrap();

        assert!(promote_local_dora_python_wheel(&staging_dir, &shared_dir).is_err());
    }

    #[test]
    fn promote_replaces_shared_dir_that_lost_its_wheel() {
        let temp = tempfile::tempdir().unwrap();
        let staging_dir = temp.path().join("build-1");
        let shared_dir = temp.path().join("fp-abc");
        let wheel_name = "dora_rs-0.4.0-cp37-abi3-manylinux_2_34_x86_64.whl";
        fs::create_dir_all(&staging_dir).unwrap();
        fs::write(staging_dir.join(wheel_name), "").unwrap();
        fs::create_dir_all(&shared_dir).unwrap();
        fs::write(shared_dir.join("stray.txt"), "").unwrap();

        let wheel = promote_local_dora_python_wheel(&staging_dir, &shared_dir).unwrap();

        assert_eq!(wheel, shared_dir.join(wheel_name));
        assert!(wheel.is_file());
        assert!(!shared_dir.join("stray.txt").exists());
        assert!(!staging_dir.exists());
    }

    #[test]
    fn finds_built_local_runtime_wheel() {
        let temp = tempfile::tempdir().unwrap();
        let old_wheel = temp
            .path()
            .join("dora_rs-0.2.0-cp37-abi3-manylinux_2_34_x86_64.whl");
        let wheel = temp
            .path()
            .join("dora_rs-0.2.1-cp37-abi3-manylinux_2_34_x86_64.whl");
        fs::write(old_wheel, "").unwrap();
        fs::write(&wheel, "").unwrap();
        fs::write(temp.path().join("other-0.1.0-py3-none-any.whl"), "").unwrap();

        assert_eq!(find_local_dora_python_wheel(temp.path()).unwrap(), wheel);
    }

    #[test]
    fn local_runtime_source_fingerprint_changes_when_source_changes() {
        let temp = tempfile::tempdir().unwrap();
        let package_dir = create_local_python_package(temp.path());

        let before = local_dora_python_source_fingerprint(&package_dir).unwrap();
        fs::write(
            package_dir.join("dora").join("__init__.py"),
            "version = 2\n",
        )
        .unwrap();
        let after = local_dora_python_source_fingerprint(&package_dir).unwrap();

        assert_ne!(before, after);
    }

    #[test]
    fn local_runtime_source_fingerprint_ignores_build_outputs() {
        let temp = tempfile::tempdir().unwrap();
        let package_dir = create_local_python_package(temp.path());

        let before = local_dora_python_source_fingerprint(&package_dir).unwrap();
        let target_dir = temp.path().join("target").join("debug");
        fs::create_dir_all(&target_dir).unwrap();
        fs::write(target_dir.join("generated.rs"), "fn ignored() {}\n").unwrap();
        let dora_dir = temp.path().join(".dora").join("python-envs");
        fs::create_dir_all(&dora_dir).unwrap();
        fs::write(dora_dir.join("marker.py"), "ignored = True\n").unwrap();
        let after = local_dora_python_source_fingerprint(&package_dir).unwrap();

        assert_eq!(before, after);
    }

    #[test]
    fn stale_wheel_dirs_are_removed_without_touching_unmanaged_entries() {
        let temp = tempfile::tempdir().unwrap();
        let shared_dir = temp.path().join("fp-old");
        let staging_dir = temp.path().join("build-1");
        let legacy_process_dir = temp.path().join("process-7");
        let unrelated_dir = temp.path().join("not-a-wheel-dir");
        for dir in [
            &shared_dir,
            &staging_dir,
            &legacy_process_dir,
            &unrelated_dir,
        ] {
            fs::create_dir_all(dir).unwrap();
        }

        cleanup_stale_local_dora_python_wheel_dirs(temp.path(), Duration::ZERO);

        assert!(!shared_dir.exists());
        assert!(!staging_dir.exists());
        assert!(!legacy_process_dir.exists());
        assert!(unrelated_dir.exists());
    }

    #[test]
    fn fresh_wheel_dirs_survive_cleanup() {
        let temp = tempfile::tempdir().unwrap();
        let shared_dir = temp.path().join("fp-current");
        let staging_dir = temp.path().join("build-1");
        fs::create_dir_all(&shared_dir).unwrap();
        fs::create_dir_all(&staging_dir).unwrap();

        cleanup_stale_local_dora_python_wheel_dirs(temp.path(), Duration::from_secs(3600));

        assert!(shared_dir.exists());
        assert!(staging_dir.exists());
    }

    #[test]
    fn local_runtime_marker_must_match_fingerprint() {
        let temp = tempfile::tempdir().unwrap();
        let env_dir = temp.path().join("env");
        fs::create_dir_all(&env_dir).unwrap();

        assert!(!local_runtime_marker_matches(&env_dir, "fingerprint-a"));

        write_local_runtime_marker(&env_dir, "fingerprint-a").unwrap();
        assert!(local_runtime_marker_matches(&env_dir, "fingerprint-a"));
        assert!(!local_runtime_marker_matches(&env_dir, "fingerprint-b"));
    }

    #[tokio::test]
    async fn keeps_draining_stdout_after_stderr_closes() {
        run_forward_output_test(true).await;
    }

    #[tokio::test]
    async fn keeps_draining_stderr_after_stdout_closes() {
        run_forward_output_test(false).await;
    }

    async fn run_forward_output_test(stdout_stays_open: bool) {
        let (stdout_reader, mut stdout_writer) = tokio::io::duplex(64);
        let (stderr_reader, mut stderr_writer) = tokio::io::duplex(64);
        let (tx, mut rx) = tokio::sync::mpsc::channel(16);

        let forward_task = tokio::spawn(forward_build_output(
            BufReader::new(stdout_reader),
            BufReader::new(stderr_reader),
            tx,
        ));

        let writer_task = tokio::spawn(async move {
            if stdout_stays_open {
                stderr_writer
                    .shutdown()
                    .await
                    .expect("failed to close stderr writer");

                for index in 0..256 {
                    stdout_writer
                        .write_all(format!("line-{index}\n").as_bytes())
                        .await
                        .expect("failed to write test line");
                }
                stdout_writer
                    .shutdown()
                    .await
                    .expect("failed to close stdout writer");
            } else {
                stdout_writer
                    .shutdown()
                    .await
                    .expect("failed to close stdout writer");

                for index in 0..256 {
                    stderr_writer
                        .write_all(format!("line-{index}\n").as_bytes())
                        .await
                        .expect("failed to write test line");
                }
                stderr_writer
                    .shutdown()
                    .await
                    .expect("failed to close stderr writer");
            }
        });

        let collect_task = tokio::spawn(async move {
            let mut lines = Vec::new();
            while let Some(line) = rx.recv().await {
                lines.push(line.expect("unexpected line forwarding error"));
            }
            lines
        });

        writer_task.await.expect("writer task failed");
        forward_task.await.expect("forward task failed");
        let lines = collect_task.await.expect("collector task failed");

        assert_eq!(lines.len(), 256);
        assert_eq!(lines.first().map(String::as_str), Some("line-0"));
        assert_eq!(lines.last().map(String::as_str), Some("line-255"));
    }

    fn create_local_python_package(root: &std::path::Path) -> std::path::PathBuf {
        fs::write(root.join("Cargo.toml"), "[workspace]\n").unwrap();
        fs::write(root.join("Cargo.lock"), "# lock\n").unwrap();
        let package_dir = root.join("apis").join("python").join("node");
        fs::create_dir_all(package_dir.join("dora")).unwrap();
        fs::create_dir_all(package_dir.join("src")).unwrap();
        fs::write(
            package_dir.join("pyproject.toml"),
            "[project]\nname = \"dora-rs\"\n",
        )
        .unwrap();
        fs::write(
            package_dir.join("Cargo.toml"),
            "[package]\nname = \"node\"\n",
        )
        .unwrap();
        fs::write(
            package_dir.join("dora").join("__init__.py"),
            "version = 1\n",
        )
        .unwrap();
        fs::write(
            package_dir.join("src").join("lib.rs"),
            "pub fn marker() {}\n",
        )
        .unwrap();
        package_dir
    }
}
