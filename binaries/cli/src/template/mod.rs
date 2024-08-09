use std::path::PathBuf;

mod c;
mod cxx;
mod python;
mod rust;

#[derive(Debug, clap::Args)]
pub struct CreateArgs {
    /// The entity that should be created
    #[clap(long, value_enum, default_value_t = Kind::Dataflow)]
    pub kind: Kind,
    /// The programming language that should be used
    #[clap(long, value_enum, default_value_t = Lang::Rust)]
    pub lang: Lang,
    /// Desired name of the entity
    pub name: String,
    /// Where to create the entity
    #[clap(hide = true)]
    pub path: Option<PathBuf>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
pub enum Kind {
    Dataflow,
    CustomNode,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
pub enum Lang {
    Rust,
    Python,
    C,
    Cxx,
}

pub fn create(args: CreateArgs, use_path_deps: bool) -> eyre::Result<()> {
    match args.lang {
        Lang::Rust => rust::create(args, use_path_deps),
        Lang::Python => python::create(args),
        Lang::C => c::create(args, use_path_deps),
        Lang::Cxx => cxx::create(args, use_path_deps),
    }
}
