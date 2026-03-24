use crate::command::Executable;

mod add;
mod info;
mod list;
mod remove;
mod restart;
mod stop;

pub use add::Add;
pub use info::Info;
pub use list::List;
pub use remove::Remove;
pub use restart::Restart;
pub use stop::Stop;

/// Manage and inspect dataflow nodes.
#[derive(Debug, clap::Subcommand)]
pub enum Node {
    List(List),
    Info(Info),
    Add(Add),
    Remove(Remove),
    Restart(Restart),
    Stop(Stop),
}

impl Executable for Node {
    fn execute(self) -> eyre::Result<()> {
        match self {
            Node::List(cmd) => cmd.execute(),
            Node::Info(cmd) => cmd.execute(),
            Node::Add(cmd) => cmd.execute(),
            Node::Remove(cmd) => cmd.execute(),
            Node::Restart(cmd) => cmd.execute(),
            Node::Stop(cmd) => cmd.execute(),
        }
    }
}
