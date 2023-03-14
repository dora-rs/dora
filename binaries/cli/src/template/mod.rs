mod c;
mod cxx;
mod python;
mod rust;

pub fn create(args: crate::CommandNew, use_path_deps: bool) -> eyre::Result<()> {
    match args.lang {
        crate::Lang::Rust => rust::create(args, use_path_deps),
        crate::Lang::Python => python::create(args),
        crate::Lang::C => c::create(args),
        crate::Lang::Cxx => cxx::create(args),
    }
}
