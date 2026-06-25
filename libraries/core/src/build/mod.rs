pub use git::GitManager;
pub use logger::{BuildLogger, LogLevelOrStdout};

use std::{
    collections::BTreeMap,
    future::Future,
    path::{Path, PathBuf},
};

use crate::descriptor::ResolvedNode;
use dora_message::{
    SessionId,
    common::{GitSource, LogLevel},
    descriptor::{CoreNodeKind, CustomNode, EnvValue, OperatorSource},
    id::NodeId,
};
use eyre::Context;

use build_command::{prepare_managed_python_env, run_build_command};
use git::GitFolder;

mod build_command;
mod git;
mod logger;

#[derive(Clone)]
pub struct Builder {
    pub session_id: SessionId,
    pub base_working_dir: PathBuf,
    pub uv: bool,
}

impl Builder {
    pub async fn build_node<L>(
        self,
        node: ResolvedNode,
        git: Option<GitSource>,
        prev_git: Option<PrevGitSource>,
        mut logger: L,
        git_manager: &mut GitManager,
    ) -> eyre::Result<impl Future<Output = eyre::Result<BuiltNode>> + use<L>>
    where
        L: BuildLogger,
    {
        let prepared_git = if let Some(GitSource {
            repo,
            commit_hash,
            subdir,
            hub: _,
        }) = git
        {
            if let Some(subdir) = &subdir {
                validate_subdir(subdir)?;
            }
            let target_dir = self.base_working_dir.join("git");
            let git_folder = git_manager.choose_clone_dir(
                self.session_id,
                repo,
                commit_hash,
                prev_git,
                &target_dir,
            )?;
            Some((git_folder, subdir))
        } else {
            None
        };

        let task = async move { self.build_node_inner(node, &mut logger, prepared_git).await };
        Ok(task)
    }

    async fn build_node_inner(
        self,
        node: ResolvedNode,
        logger: &mut impl BuildLogger,
        git_folder: Option<(GitFolder, Option<String>)>,
    ) -> eyre::Result<BuiltNode> {
        logger.log_message(LogLevel::Debug, "building node").await;
        let python_env_dir;
        let node_working_dir = match &node.kind {
            CoreNodeKind::Custom(n) => {
                let node_working_dir = match git_folder {
                    Some((git_folder, subdir)) => {
                        let clone_dir = git_folder.prepare(logger).await?;
                        tracing::warn!(
                            "using git clone directory as working dir: \
                            this behavior is unstable and might change"
                        );
                        match subdir {
                            // monorepo node: build, env prep, and spawn root
                            // at `<clone>/<subdir>` (validated above)
                            Some(subdir) => confine_subdir(&clone_dir, &subdir)?,
                            None => clone_dir,
                        }
                    }
                    None => self.base_working_dir,
                };

                // `managed_python_env_dir` itself returns None for script-only
                // Python custom nodes (no `build:`) — those run against the
                // caller's ambient uv env.
                python_env_dir = managed_python_env_dir(&node, &node_working_dir);
                if self.uv
                    && let Some(python_env_dir) = &python_env_dir
                {
                    prepare_python_env(
                        logger,
                        &node.env,
                        node_working_dir.clone(),
                        python_env_dir.clone(),
                    )
                    .await?;
                }

                if let Some(build) = &n.build {
                    build_node(
                        logger,
                        &node.env,
                        node_working_dir.clone(),
                        build,
                        self.uv,
                        python_env_dir.clone(),
                    )
                    .await?;
                }
                node_working_dir
            }
            CoreNodeKind::Runtime(n) => {
                // Runtime nodes use one managed Python env per node; prepare it once before
                // running any Python operator build commands.
                python_env_dir = managed_python_env_dir(&node, &self.base_working_dir);
                if self.uv
                    && let Some(python_env_dir) = &python_env_dir
                {
                    prepare_python_env(
                        logger,
                        &node.env,
                        self.base_working_dir.clone(),
                        python_env_dir.clone(),
                    )
                    .await?;
                }
                // run build commands
                for operator in &n.operators {
                    if let Some(build) = &operator.config.build {
                        build_node(
                            logger,
                            &node.env,
                            self.base_working_dir.clone(),
                            build,
                            self.uv,
                            python_env_dir.clone(),
                        )
                        .await?;
                    }
                }
                self.base_working_dir.clone()
            }
        };
        Ok(BuiltNode {
            node_working_dir,
            python_env_dir,
        })
    }
}

async fn build_node(
    logger: &mut impl BuildLogger,
    node_env: &Option<BTreeMap<String, EnvValue>>,
    working_dir: PathBuf,
    build: &String,
    uv: bool,
    python_env_dir: Option<PathBuf>,
) -> eyre::Result<()> {
    logger
        .log_message(
            LogLevel::Info,
            format!(
                "running build command: `{build}` in {}",
                working_dir.display()
            ),
        )
        .await;
    let build = build.to_owned();
    let node_env = node_env.clone();
    let mut logger = logger.try_clone().await.context("failed to clone logger")?;
    let (stdout_tx, mut stdout) = tokio::sync::mpsc::channel(10);
    let task = tokio::spawn(async move {
        run_build_command(
            &build,
            &working_dir,
            uv,
            python_env_dir,
            &node_env,
            stdout_tx,
        )
        .await
        .context("build command failed")
    });
    let stdout_task = tokio::spawn(async move {
        while let Some(line) = stdout.recv().await {
            logger
                .log_stdout(line.unwrap_or_else(|err| format!("io err: {}", err.kind())))
                .await;
        }
    });
    stdout_task.await?;
    task.await??;

    Ok(())
}

async fn prepare_python_env(
    logger: &mut impl BuildLogger,
    node_env: &Option<BTreeMap<String, EnvValue>>,
    working_dir: PathBuf,
    python_env_dir: PathBuf,
) -> eyre::Result<()> {
    logger
        .log_message(
            LogLevel::Info,
            format!(
                "preparing managed Python env in {}",
                python_env_dir.display()
            ),
        )
        .await;
    let node_env = node_env.clone();
    let mut logger = logger.try_clone().await.context("failed to clone logger")?;
    let (stdout_tx, mut stdout) = tokio::sync::mpsc::channel(10);
    let task = tokio::spawn(async move {
        prepare_managed_python_env(&working_dir, &python_env_dir, &node_env, stdout_tx)
            .await
            .context("managed Python env preparation failed")
    });
    let stdout_task = tokio::spawn(async move {
        while let Some(line) = stdout.recv().await {
            logger
                .log_stdout(line.unwrap_or_else(|err| format!("io err: {}", err.kind())))
                .await;
        }
    });
    stdout_task.await?;
    task.await??;

    Ok(())
}

pub struct BuiltNode {
    pub node_working_dir: PathBuf,
    pub python_env_dir: Option<PathBuf>,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct BuildInfo {
    pub node_working_dirs: BTreeMap<NodeId, PathBuf>,
    #[serde(default)]
    pub python_env_dirs: BTreeMap<NodeId, PathBuf>,
    /// Nodes that must be spawned with confined path resolution (no ambient
    /// `$PATH` fallback) — set for hub-sourced nodes (spec §11).
    #[serde(default)]
    pub confined_nodes: std::collections::BTreeSet<NodeId>,
}

/// Validate a git source `subdir`: a relative path confined to the clone
/// (no absolute, no `..`, no leading `-`, printable characters only). The
/// value can come from an untrusted hub index entry.
pub fn validate_subdir(subdir: &str) -> eyre::Result<()> {
    let confined = !subdir.is_empty()
        && !subdir.starts_with(['/', '\\', '-', '~'])
        && !subdir.split(['/', '\\']).any(|c| c == "..")
        && !subdir.contains(|c: char| c.is_control())
        && (subdir.len() < 2 || subdir.as_bytes()[1] != b':');
    if confined {
        Ok(())
    } else {
        eyre::bail!(
            "invalid git source subdir `{}`: must be a relative path inside \
             the repository",
            subdir
                .chars()
                .filter(|c| !c.is_control())
                .collect::<String>()
        )
    }
}

/// Resolve `<clone_dir>/<subdir>` and confine it to the clone, returning the
/// real (canonical) directory.
///
/// [`validate_subdir`] is lexical only; `is_dir()` follows in-repo symlinks, so
/// a package shipping `node-hub/x -> /etc` could otherwise root the build (and
/// the entrypoint `confine` check) outside the clone. Requiring the canonical
/// path to stay under the canonical clone closes that escape.
fn confine_subdir(clone_dir: &Path, subdir: &str) -> eyre::Result<PathBuf> {
    let dir = clone_dir.join(subdir);
    if !dir.is_dir() {
        eyre::bail!(
            "git source has no `{subdir}` directory in `{}`",
            clone_dir.display()
        );
    }
    let real = dir
        .canonicalize()
        .with_context(|| format!("failed to resolve git source subdir `{subdir}`"))?;
    let real_clone = clone_dir
        .canonicalize()
        .context("failed to resolve git clone directory")?;
    if !real.starts_with(&real_clone) {
        eyre::bail!("git source subdir `{subdir}` escapes the repository (symlink?)");
    }
    Ok(real)
}

/// Computes the managed Python env directory for `node`, if it needs one.
///
/// Returns `Some(working_dir/.dora/python-envs/<node-id>)` when the node
/// actually needs Dora to prepare a venv for it:
///
/// - Python custom node (`.py` extension) **with a `build:` block** — the
///   build commands install deps into the managed env, which the runtime
///   then reuses.
/// - Runtime node with at least one Python operator **that does not use
///   `conda_env:`** — operators with `conda_env:` manage their own Python
///   via `conda run`, and overlaying a uv venv on top would mix two env
///   managers (the conda interpreter would run with `VIRTUAL_ENV` pointing
///   at the wrong place and PATH searching the uv venv first).
///
/// Returns `None` for script-only Python custom nodes (no `build:` block) —
/// those run against the caller's ambient `uv` environment by design —
/// for Python operators that pin `conda_env:`, and for non-Python nodes.
///
/// The `node_id` segment is what gives each node its own venv even when nodes
/// share a `working_dir`. Runtime nodes always do, and custom nodes without
/// a git source also fall back to the shared `base_working_dir`. Without that
/// segment, parallel builds would race on `uv venv --clear` against the same
/// directory and silently clobber each other's envs.
pub fn managed_python_env_dir(node: &ResolvedNode, node_working_dir: &Path) -> Option<PathBuf> {
    node_requires_managed_python_env(node).then(|| {
        let envs_base = node_working_dir.join(".dora").join("python-envs");
        let env_dir = envs_base.join(node.id.as_ref());
        // Defense-in-depth: the node id must not escape the python-envs/ directory.
        // validate_node_id already rejects leading-dot ids (`.`, `..`, `.hidden`),
        // so this assertion should never fire in practice — it guards against future
        // callers that bypass validation.
        debug_assert!(
            !env_dir.components().any(|c| matches!(c, std::path::Component::ParentDir)),
            "node id '{id}' contains a parent-dir segment (env_dir={env_dir:?})",
            id = node.id,
        );
        env_dir
    })
}

/// Returns the bin directory inside an env dir (`bin/` on Unix, `Scripts/` on Windows).
pub fn managed_python_bin_dir(python_env_dir: &Path) -> PathBuf {
    if cfg!(windows) {
        python_env_dir.join("Scripts")
    } else {
        python_env_dir.join("bin")
    }
}

/// Returns the interpreter path inside an env dir (`bin/python` or `Scripts/python.exe`).
pub fn managed_python_interpreter(python_env_dir: &Path) -> PathBuf {
    let executable = if cfg!(windows) {
        "python.exe"
    } else {
        "python"
    };
    managed_python_bin_dir(python_env_dir).join(executable)
}

fn node_requires_managed_python_env(node: &ResolvedNode) -> bool {
    match &node.kind {
        // Script-only Python custom nodes (no `build:`) run against the
        // caller's ambient `uv` env — they have no deps for Dora to install
        // and the spawn path uses `uv run python` rather than the managed
        // interpreter for them.
        CoreNodeKind::Custom(custom) => is_python_custom_node(custom) && custom.build.is_some(),
        // Python operators with `conda_env:` manage their own Python via
        // `conda run`; preparing a uv venv and injecting `VIRTUAL_ENV`/`PATH`
        // would mix two environment managers and make subprocesses resolve
        // from the wrong env. Only ask for a managed venv when the operator
        // is plain Python (no conda_env).
        CoreNodeKind::Runtime(runtime) => runtime.operators.iter().any(|operator| {
            matches!(
                &operator.config.source,
                OperatorSource::Python(py) if py.conda_env.is_none()
            )
        }),
    }
}

fn is_python_custom_node(custom: &CustomNode) -> bool {
    Path::new(&custom.path)
        .extension()
        .and_then(|ext| ext.to_str())
        .is_some_and(|ext| ext.eq_ignore_ascii_case("py"))
}

pub struct PrevGitSource {
    pub git_source: GitSource,
    /// `True` if any nodes of this dataflow still require the source for building.
    pub still_needed_for_this_build: bool,
}

#[cfg(test)]
mod tests {
    use super::{
        managed_python_bin_dir, managed_python_env_dir, managed_python_interpreter, validate_subdir,
    };
    use crate::descriptor::ResolvedNode;
    use std::path::PathBuf;

    /// Deserialize from YAML to avoid coupling tests to ResolvedNode's exact
    /// field list — `#[serde(default)]` on the optional fields handles drift.
    fn resolved_node_from_yaml(yaml: &str) -> ResolvedNode {
        serde_yaml::from_str(yaml).expect("test node yaml should parse")
    }

    #[test]
    fn assigns_managed_env_dir_to_build_backed_python_custom_nodes() {
        let node = resolved_node_from_yaml(
            "id: python-custom\n\
             custom:\n  \
             path: node.py\n  \
             source: Local\n  \
             build: pip install -r requirements.txt\n",
        );
        let env_dir = managed_python_env_dir(&node, &PathBuf::from("workdir"))
            .expect("build-backed python custom node should get managed env dir");
        assert_eq!(
            env_dir,
            PathBuf::from("workdir/.dora/python-envs/python-custom")
        );
    }

    #[test]
    fn skips_managed_env_dir_for_script_only_python_custom_nodes() {
        // Script-only Python custom nodes (no `build:`) intentionally run
        // against the caller's ambient uv env — both the build flow and the
        // spawn flow short-circuit them. The predicate must match so the
        // daemon's fail-closed guard does not over-fire under `dora start --uv`.
        let node = resolved_node_from_yaml(
            "id: script-only-python\n\
             custom:\n  \
             path: node.py\n  \
             source: Local\n",
        );
        assert!(managed_python_env_dir(&node, &PathBuf::from("workdir")).is_none());
    }

    #[test]
    fn env_dir_includes_node_id_so_parallel_builds_do_not_race() {
        // Two Python nodes sharing the same working_dir (e.g. two runtime nodes
        // in one dataflow, or two custom nodes without git sources) must get
        // distinct env dirs; without per-node identification, parallel builds
        // would race on `uv venv --clear` against the same directory.
        let node_a = resolved_node_from_yaml(
            "id: node-a\n\
             custom:\n  \
             path: a.py\n  \
             source: Local\n  \
             build: pip install foo\n",
        );
        let node_b = resolved_node_from_yaml(
            "id: node-b\n\
             custom:\n  \
             path: b.py\n  \
             source: Local\n  \
             build: pip install bar\n",
        );
        let shared = PathBuf::from("shared-workdir");
        let env_a = managed_python_env_dir(&node_a, &shared).expect("node-a env");
        let env_b = managed_python_env_dir(&node_b, &shared).expect("node-b env");
        assert_ne!(env_a, env_b);
    }

    #[test]
    fn skips_managed_env_dir_for_runtime_python_operators_with_conda_env() {
        // Python operators with `conda_env:` manage their own Python via
        // `conda run` at spawn time. Layering a uv venv on top mixes two
        // env managers and breaks subprocess resolution. Build must skip
        // venv creation and spawn must skip env injection.
        let node = resolved_node_from_yaml(
            "id: runtime-with-conda\n\
             operators:\n  \
             - id: op\n    \
             python:\n      \
             source: op.py\n      \
             conda_env: my-env\n",
        );
        assert!(managed_python_env_dir(&node, &PathBuf::from("workdir")).is_none());
    }

    #[test]
    fn assigns_managed_env_dir_for_runtime_python_operators_without_conda_env() {
        let node = resolved_node_from_yaml(
            "id: runtime-no-conda\n\
             operators:\n  \
             - id: op\n    \
             python: op.py\n",
        );
        let env_dir = managed_python_env_dir(&node, &PathBuf::from("workdir"))
            .expect("runtime with plain python operator should get managed env dir");
        assert_eq!(
            env_dir,
            PathBuf::from("workdir/.dora/python-envs/runtime-no-conda")
        );
    }

    #[test]
    fn skips_managed_env_dir_for_non_python_custom_nodes() {
        let node = resolved_node_from_yaml(
            "id: rust-custom\n\
             custom:\n  \
             path: target/release/my-node\n  \
             source: Local\n",
        );
        assert!(managed_python_env_dir(&node, &PathBuf::from("workdir")).is_none());
    }

    #[test]
    fn build_info_deserializes_without_python_env_dirs() {
        let build_info: super::BuildInfo =
            serde_yaml::from_str("node_working_dirs: {}\n").expect("build info should parse");
        assert!(build_info.python_env_dirs.is_empty());
    }

    #[test]
    fn managed_python_helpers_use_platform_layout() {
        let env_dir = PathBuf::from("env-dir");
        let expected_bin_dir = if cfg!(windows) {
            PathBuf::from("env-dir/Scripts")
        } else {
            PathBuf::from("env-dir/bin")
        };
        let expected_interpreter = if cfg!(windows) {
            PathBuf::from("env-dir/Scripts/python.exe")
        } else {
            PathBuf::from("env-dir/bin/python")
        };

        assert_eq!(managed_python_bin_dir(&env_dir), expected_bin_dir);
        assert_eq!(managed_python_interpreter(&env_dir), expected_interpreter);
    }

    #[test]
    fn subdir_validation_confines_to_the_clone() {
        for good in ["node-hub/dora-yolo", "src", "a/b/c", "a.b"] {
            assert!(validate_subdir(good).is_ok(), "{good} should be valid");
        }
        for bad in [
            "",
            "/abs",
            "\\abs",
            "../outside",
            "a/../../b",
            "a\\..\\b",
            "-flag",
            "~home",
            "C:\\windows",
            "a\x07b",
        ] {
            assert!(validate_subdir(bad).is_err(), "{bad:?} should be invalid");
        }
    }

    #[cfg(unix)]
    #[test]
    fn confine_subdir_rejects_symlink_escape() {
        let clone = tempfile::tempdir().unwrap();
        let outside = tempfile::tempdir().unwrap();
        // a real in-repo subdir resolves to itself
        std::fs::create_dir(clone.path().join("src")).unwrap();
        assert!(super::confine_subdir(clone.path(), "src").is_ok());
        // an in-repo symlink pointing outside the clone (`escape -> <outside>`)
        // passes `validate_subdir` (no `..`/absolute) and `is_dir()`, but must
        // be rejected by the canonical-containment check
        std::os::unix::fs::symlink(outside.path(), clone.path().join("escape")).unwrap();
        assert!(validate_subdir("escape").is_ok());
        let err = super::confine_subdir(clone.path(), "escape").unwrap_err();
        assert!(err.to_string().contains("escapes the repository"), "{err}");
    }
}
