pub use crate::command::topic::{echo::Echo, hz::Hz, list::List};

mod echo;
mod hz;
mod list;
mod selector;

/// Manage and inspect dataflow topics.
#[enum_dispatch::enum_dispatch(Executable)]
#[derive(Debug, clap::Subcommand)]
pub enum Topic {
    List(List),
    Echo(Echo),
    Hz(Hz),
}
