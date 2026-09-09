use eyre::{ContextCompat, bail};
use std::path::Path;

mod c;
mod cxx;
mod python;
mod rust;

/// Validates a `dora new` project/node name.
///
/// The name is substituted, unescaped, into generated `CMakeLists.txt`
/// (`project(___name___ ...)`, `add_executable(___name___ ...)`) and
/// `Cargo.toml`/`pyproject.toml` (`name = "___name___"`), and used as a
/// filesystem path component. An allow-list of ASCII letters, digits, `-`,
/// and `_` keeps it safe in all three: none of those characters carry
/// syntactic meaning in CMake's unquoted argument list, a quoted TOML
/// string, or a path.
///
/// `allow_spaces` widens the set for the Python backend only, which
/// normalizes spaces to `-`/`_` before the name touches a file and never
/// emits CMake -- so a bare space can't break an unquoted CMake argument
/// there the way it would for C/C++/Rust.
pub(crate) fn validate_name(name: &str, allow_spaces: bool) -> eyre::Result<()> {
    if name.is_empty() {
        bail!("name must not be empty");
    }
    if let Some(c) = name.chars().find(|&c| {
        !c.is_ascii_alphanumeric() && c != '_' && c != '-' && !(allow_spaces && c == ' ')
    }) {
        bail!(
            "name contains invalid character '{c}' -- only ASCII letters, digits, `-`, `_`{} are allowed",
            if allow_spaces { ", and spaces" } else { "" }
        );
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
        assert!(validate_name("my-node_1", false).is_ok());
        assert!(validate_name("MyNode123", false).is_ok());
    }

    #[test]
    fn validate_name_rejects_empty() {
        assert!(validate_name("", false).is_err());
        assert!(validate_name("", true).is_err());
    }

    #[test]
    fn validate_name_rejects_cmake_breaking_space_unless_allowed() {
        assert!(validate_name("my node", false).is_err());
        assert!(validate_name("my node", true).is_ok());
    }

    #[test]
    fn validate_name_rejects_toml_breaking_characters_even_with_spaces_allowed() {
        for name in ["foo\"bar", "foo\\bar", "foo/bar", "foo#bar", "café"] {
            assert!(
                validate_name(name, false).is_err(),
                "expected {name:?} to be rejected"
            );
            assert!(
                validate_name(name, true).is_err(),
                "expected {name:?} to be rejected even with spaces allowed"
            );
        }
    }
}
