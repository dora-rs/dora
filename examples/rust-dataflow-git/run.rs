use dora_cli::{build, run};
use eyre::{Context, ContextCompat};
use std::{
    fs,
    path::{Path, PathBuf},
    process::Command,
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
    eprintln!("Resolving dataflow template: {}", dataflow);
    let resolved_dataflow = resolve_dataflow_template(Path::new(&dataflow))?;
    eprintln!("Resolved dataflow path: {}", resolved_dataflow);

    build(resolved_dataflow.clone(), None, None, false, true)?;

    run(resolved_dataflow, false)?;

    Ok(())
}

fn resolve_dataflow_template(dataflow_path: &Path) -> eyre::Result<String> {
    let contents = fs::read_to_string(dataflow_path)
        .with_context(|| format!("failed to read {}", dataflow_path.display()))?;

    if !contents.contains("__DORA_GIT_URL__") && !contents.contains("__DORA_GIT_REV__") {
        return Ok(dataflow_path.to_string_lossy().into_owned());
    }

    let repo_root = git_output(
        Path::new(env!("CARGO_MANIFEST_DIR")),
        &["rev-parse", "--show-toplevel"],
    )?;
    let repo_root = PathBuf::from(repo_root.trim());
    let path_str = repo_root.to_string_lossy().replace('\\', "/");
    let git_url = if path_str.starts_with('/') {
        format!("file://{}", path_str)
    } else {
        // Windows drive paths (e.g., C:/foo)
        format!("file:///{}", path_str)
    };
    let git_rev = git_output(&repo_root, &["rev-parse", "HEAD"])?;

    let resolved = contents
        .replace("__DORA_GIT_URL__", &git_url)
        .replace("__DORA_GIT_REV__", git_rev.trim());

    let output_path = resolved_dataflow_path(dataflow_path)?;
    if let Some(parent) = output_path.parent() {
        fs::create_dir_all(parent)
            .with_context(|| format!("failed to create {}", parent.display()))?;
    }
    fs::write(&output_path, resolved)
        .with_context(|| format!("failed to write {}", output_path.display()))?;

    Ok(output_path.to_string_lossy().into_owned())
}

fn resolved_dataflow_path(dataflow_path: &Path) -> eyre::Result<PathBuf> {
    let stem = dataflow_path
        .file_stem()
        .and_then(|stem| stem.to_str())
        .context("dataflow path has no valid file stem")?;
    Ok(dataflow_path
        .with_file_name("out")
        .join(format!("{stem}.resolved.yml")))
}

fn git_output(repo_root: &Path, args: &[&str]) -> eyre::Result<String> {
    let output = Command::new("git")
        .args(args)
        .current_dir(repo_root)
        .output()
        .with_context(|| format!("failed to run git {}", args.join(" ")))?;

    if !output.status.success() {
        eyre::bail!(
            "git {} failed: {}",
            args.join(" "),
            String::from_utf8_lossy(&output.stderr).trim()
        );
    }

    String::from_utf8(output.stdout).context("git output was not valid utf-8")
}
