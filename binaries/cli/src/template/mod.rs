use eyre::{ContextCompat, bail};
use std::path::Path;

mod c;
mod cxx;
mod python;
mod rust;

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

/// Validate a node or dataflow `name` before it is substituted into generated
/// files.
///
/// The name is interpolated raw into syntax-sensitive contexts — unquoted
/// CMake arguments (`project(___name___ ...)`), TOML strings
/// (`name = "___name___"`), YAML scalars and directory names — so anything
/// outside `[A-Za-z0-9_-]` can produce a broken, unbuildable project
/// (see issue #3440). The first character must be a letter so the name is
/// also usable as a bare CMake argument, a Cargo package name and a Python
/// module name; the last character must be a letter or digit so the name is
/// a valid Python distribution name (PEP 508). Names cargo reserves for its
/// own build directories are rejected too, since the Rust node template
/// makes the package name the implicit binary target name.
const RESERVED_CARGO_TARGET_NAMES: &[&str] = &["build", "deps", "examples", "incremental"];

fn validate_name(name: &str, kind: &str) -> eyre::Result<()> {
    let mut chars = name.chars();
    let valid = !name.is_empty()
        && chars.next().is_some_and(|c| c.is_ascii_alphabetic())
        && chars.all(|c| c.is_ascii_alphanumeric() || c == '-' || c == '_')
        && name.ends_with(|c: char| c.is_ascii_alphanumeric());
    if !valid {
        bail!(
            "{kind} name `{name}` is invalid: use only ASCII letters, digits, `-` and `_`, starting and ending with a letter or digit"
        );
    }
    if RESERVED_CARGO_TARGET_NAMES.contains(&name) {
        bail!("{kind} name `{name}` is invalid: cargo forbids target names that collide with its build directories");
    }
    Ok(())
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
    fn validate_name_accepts_safe_names() {
        for name in ["talker_1", "my-node", "node2", "a", "A-_0"] {
            validate_name(name, "node").unwrap_or_else(|_| panic!("should accept `{name}`"));
            validate_name(name, "dataflow").unwrap_or_else(|_| panic!("should accept `{name}`"));
        }
    }

    #[test]
    fn validate_name_rejects_names_that_break_generated_files() {
        // Each of these passed the old validation but produced broken
        // generated build files (see issue #3440).
        for name in [
            "",
            "my node",
            "foo\"bar",
            "a/b",
            "h\u{e9}llo",
            "(x)",
            "a#b",
            "a;b",
            "-lead",
            "9lives",
            "_",
            "a_b ",
            // Trailing `-`/`_` fails PEP 508 for generated Python projects.
            "foo-",
            "foo_",
            // Cargo forbids these as (implicit binary) target names since
            // they collide with its build directories.
            "build",
            "deps",
            "examples",
            "incremental",
        ] {
            assert!(
                validate_name(name, "node").is_err(),
                "should reject node name `{name}`"
            );
            assert!(
                validate_name(name, "dataflow").is_err(),
                "should reject dataflow name `{name}`"
            );
        }
    }
}
