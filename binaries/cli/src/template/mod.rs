mod c;
mod cxx;
mod python;
mod rust;

pub fn create(args: crate::CommandNew) -> eyre::Result<()> {
    match args.lang {
        crate::Lang::Rust => rust::create(args),
        crate::Lang::Python => python::create(args),
        crate::Lang::C => c::create(args),
        crate::Lang::Cxx => todo!(),
    }
}
