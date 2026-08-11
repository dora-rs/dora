use eyre::ContextCompat;
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
}

pub fn create(args: crate::CommandNew, use_path_deps: bool) -> eyre::Result<()> {
    match args.lang {
        crate::Lang::Rust => rust::create(args, use_path_deps),
        crate::Lang::Python => python::create(args, use_path_deps),
        crate::Lang::C => c::create(args, use_path_deps),
        crate::Lang::Cxx => cxx::create(args, use_path_deps),
    }
}
