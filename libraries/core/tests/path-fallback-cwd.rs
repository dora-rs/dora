//! A module node's `path` is the file next to the module; when it is missing,
//! `path_fallback` is looked up like a top-level `path` (#3826). A file at the
//! same relative path under the current directory (another project) must not
//! hide the fallback. Its own test binary, because it changes the current
//! directory.

use dora_core::descriptor::resolve_node_path;
use std::{env::consts::EXE_SUFFIX, fs, path::Path};

fn executable(path: &Path) {
    fs::create_dir_all(path.parent().unwrap()).unwrap();
    fs::write(path, "").unwrap();
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        fs::set_permissions(path, fs::Permissions::from_mode(0o755)).unwrap();
    }
}

#[test]
fn a_file_at_the_same_path_in_the_current_directory_does_not_hide_the_fallback() {
    let dataflow = tempfile::tempdir().unwrap();
    let elsewhere = tempfile::tempdir().unwrap();
    let fallback = dataflow.path().join(format!("tool{EXE_SUFFIX}"));
    executable(&fallback);
    executable(
        &elsewhere
            .path()
            .join("modules")
            .join(format!("tool{EXE_SUFFIX}")),
    );
    std::env::set_current_dir(elsewhere.path()).unwrap();

    let resolved = resolve_node_path("modules/tool", Some("tool"), dataflow.path()).unwrap();

    assert_eq!(
        resolved.canonicalize().unwrap(),
        fallback.canonicalize().unwrap()
    );
}
