use eyre::ContextCompat;
use std::path::Path;

mod c;
mod cxx;
mod python;
mod rust;

/// Path to the dora workspace root (two levels above the CLI crate
/// manifest), used by the C/C++ templates to reference dora via path
/// dependencies when `use_path_deps` is set.
fn workspace_dir() -> eyre::Result<&'static str> {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .context("Could not get manifest parent folder")?
        .parent()
        .context("Could not get manifest grandparent folder")?
        .to_str()
        .context("dora workspace path is not valid UTF-8")
}

pub fn create(args: crate::CommandNew, use_path_deps: bool) -> eyre::Result<()> {
    match args.lang {
        crate::Lang::Rust => rust::create(args, use_path_deps),
        crate::Lang::Python => python::create(args, use_path_deps),
        crate::Lang::C => c::create(args, use_path_deps),
        crate::Lang::Cxx => cxx::create(args, use_path_deps),
    }
}
