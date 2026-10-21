use eyre::{ContextCompat, bail};
use std::path::Path;

mod c;
mod cxx;
mod python;
mod rust;

/// Validates a `dora new` project/node name.
///
/// `kind` is a short label ("node" or "dataflow") used only to give the
/// error message the right context; it does not affect what's accepted.
///
/// The name is substituted, unescaped, into generated `CMakeLists.txt`
/// (`project(___name___ ...)`, `add_executable(___name___ ...)`) and
/// `Cargo.toml`/`pyproject.toml` (`name = "___name___"`), and used as a
/// filesystem path component. An allow-list of ASCII letters, digits, `-`,
/// and `_` keeps it safe in all three: none of those characters carry
/// syntactic meaning in CMake's unquoted argument list, a quoted TOML
/// string, or a path.
///
/// `allow_spaces` widens the set for the Python `create_custom_node` path
/// only, which normalizes spaces to `-`/`_` before the name touches a file
/// and never emits CMake -- so a bare space can't break an unquoted CMake
/// argument there the way it would for C/C++/Rust. Every other call site,
/// including the Python dataflow path, passes `false`.
///
/// The name must also start with an ASCII letter, end with a letter or
/// digit, and avoid the names cargo reserves (`build`, `deps`, `examples`,
/// `incremental`).
pub(crate) fn validate_name(kind: &str, name: &str, allow_spaces: bool) -> eyre::Result<()> {
    const CARGO_RESERVED: [&str; 4] = ["build", "deps", "examples", "incremental"];

    if name.is_empty() {
        bail!("{kind} name must not be empty");
    }
    if let Some(c) = name.chars().find(|&c| {
        !c.is_ascii_alphanumeric() && c != '_' && c != '-' && !(allow_spaces && c == ' ')
    }) {
        bail!(
            "{kind} name contains invalid character '{c}' -- only ASCII letters, digits, `-`, `_`{} are allowed",
            if allow_spaces { ", and spaces" } else { "" }
        );
    }
    // A leading digit is an invalid Python module name; a leading `-`/`_` is
    // an invalid PEP 508 project and cargo package name.
    if !name.starts_with(|c: char| c.is_ascii_alphabetic()) {
        bail!("{kind} name must start with an ASCII letter");
    }
    // PEP 508 project names must end with an alphanumeric.
    if !name.ends_with(|c: char| c.is_ascii_alphanumeric()) {
        bail!("{kind} name must end with an ASCII letter or digit");
    }
    // The Rust node template makes the package name the implicit bin target,
    // and cargo rejects these names for one at manifest parse.
    if CARGO_RESERVED.contains(&name) {
        bail!("{kind} name `{name}` is reserved by cargo and cannot be used");
    }
    Ok(())
}

/// Path to the dora workspace root (two levels above the CLI crate
/// manifest), used by the C/C++ templates to reference dora via path
/// dependencies when `use_path_deps` is set.
fn workspace_dir() -> eyre::Result<String> {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .context("Could not get manifest parent folder")?
        .parent()
        .context("Could not get manifest grandparent folder")?
        .to_str()
        .context("dora workspace path is not valid UTF-8")?;
    Ok(normalize_for_cmake(dir))
}

// CMake treats `\` as a string escape character, so a raw Windows path
// (e.g. `C:\Users\...`) substituted into CMakeLists.txt fails to parse.
// Both CMake and Windows accept `/`, so normalize before substitution.
fn normalize_for_cmake(path: &str) -> String {
    path.replace('\\', "/")
}

pub fn create(args: crate::CommandNew, use_path_deps: bool) -> eyre::Result<()> {
    match args.lang {
        crate::Lang::Rust => rust::create(args, use_path_deps),
        crate::Lang::Python => python::create(args, use_path_deps),
        crate::Lang::C => c::create(args, use_path_deps),
        crate::Lang::Cxx => cxx::create(args, use_path_deps),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Uses a hardcoded Windows-style sample path rather than the live
    // `workspace_dir()` output, since on Linux/macOS CI runners the real
    // path never contains a backslash and the assertion would pass
    // trivially without exercising the normalization at all.
    #[test]
    fn normalize_for_cmake_replaces_backslashes() {
        let normalized = normalize_for_cmake(r"C:\Users\example\dora");
        assert_eq!(normalized, "C:/Users/example/dora");
        assert!(!normalized.contains('\\'));
    }

    #[test]
    fn validate_name_accepts_alnum_dash_underscore() {
        assert!(validate_name("node", "my-node_1", false).is_ok());
        assert!(validate_name("node", "MyNode123", false).is_ok());
    }

    #[test]
    fn validate_name_rejects_empty() {
        assert!(validate_name("node", "", false).is_err());
        assert!(validate_name("node", "", true).is_err());
    }

    #[test]
    fn validate_name_rejects_cmake_breaking_space_unless_allowed() {
        assert!(validate_name("node", "my node", false).is_err());
        assert!(validate_name("node", "my node", true).is_ok());
    }

    #[test]
    fn validate_name_rejects_toml_breaking_characters_even_with_spaces_allowed() {
        for name in ["foo\"bar", "foo\\bar", "foo/bar", "foo#bar", "café"] {
            assert!(
                validate_name("node", name, false).is_err(),
                "expected {name:?} to be rejected"
            );
            assert!(
                validate_name("node", name, true).is_err(),
                "expected {name:?} to be rejected even with spaces allowed"
            );
        }
    }

    // A bare `.` is syntactically harmless in both CMake's unquoted argument
    // list and a quoted TOML string -- it's rejected only because Cargo and
    // PEP 508 package names disallow it. Pre-this-PR validation used to
    // accept `my.node`, silently producing an uncompilable `Cargo.toml`.
    #[test]
    fn validate_name_rejects_dot() {
        assert!(validate_name("node", "my.node", false).is_err());
        assert!(validate_name("node", "my.node", true).is_err());
    }

    // A leading digit passes the character allow-list but is an invalid
    // Python module name (`2048/main.py` cannot be imported), so cargo/CMake
    // would accept it but the Python backend would not -- rejected uniformly.
    #[test]
    fn validate_name_rejects_leading_non_letter() {
        for name in ["2048", "_foo", "-foo"] {
            assert!(
                validate_name("node", name, false).is_err(),
                "expected {name:?} to be rejected"
            );
        }
    }

    // A trailing `-`/`_` passes the character allow-list but PEP 508 requires
    // the project name to end in an alphanumeric, so uv rejects the generated
    // pyproject.
    #[test]
    fn validate_name_rejects_trailing_dash_or_underscore() {
        for name in ["foo-", "foo_"] {
            assert!(
                validate_name("node", name, false).is_err(),
                "expected {name:?} to be rejected"
            );
        }
    }

    // These collide with cargo's build-directory names, so the Rust node
    // template's `[package] name = "<name>"` (implicit bin target) is
    // rejected by cargo at manifest parse.
    #[test]
    fn validate_name_rejects_cargo_reserved_target_names() {
        for name in ["build", "deps", "examples", "incremental"] {
            assert!(
                validate_name("node", name, false).is_err(),
                "expected {name:?} to be rejected"
            );
        }
    }

    // The internal scaffold node names must keep passing, or `dora new
    // --kind dataflow` would fail partway through creating its own nodes.
    #[test]
    fn validate_name_accepts_internal_scaffold_names() {
        for name in ["talker_1", "talker_2", "listener_1"] {
            assert!(validate_name("node", name, false).is_ok());
        }
        for name in ["talker 1", "talker 2", "listener 1"] {
            assert!(validate_name("node", name, true).is_ok());
        }
    }

    #[test]
    fn validate_name_error_message_includes_kind() {
        let err = validate_name("dataflow", "", false)
            .unwrap_err()
            .to_string();
        assert!(
            err.contains("dataflow name"),
            "expected error to mention the `kind` label, got: {err}"
        );
    }

    // Regression guard for the shared validator: proves every `create()`
    // path still calls `validate_name` at all, so a future edit can't
    // silently drop one of the eight production call sites and let
    // `dora new <bad-name>` scaffold a broken project instead of erroring
    // before any directory is created.
    #[test]
    fn create_rejects_invalid_name_for_every_lang_and_kind() {
        for lang in [
            crate::Lang::Rust,
            crate::Lang::Python,
            crate::Lang::C,
            crate::Lang::Cxx,
        ] {
            for kind in [crate::Kind::Dataflow, crate::Kind::Node] {
                let dir = tempfile::tempdir().expect("failed to create temp dir");
                let target = dir.path().join("target");
                let args = crate::CommandNew {
                    kind,
                    lang,
                    name: "bad/name".into(),
                    path: Some(target.clone()),
                };
                let result = super::create(args, false);
                assert!(
                    result.is_err(),
                    "expected {lang:?}/{kind:?} to reject a name containing `/`"
                );
                assert!(
                    !target.exists(),
                    "{lang:?}/{kind:?} created `{}` despite a rejected name",
                    target.display()
                );
            }
        }
    }
}
