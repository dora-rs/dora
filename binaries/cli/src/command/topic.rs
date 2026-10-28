use crate::command::{
    Executable,
    topic::{echo::Echo, hz::Hz, info::Info, list::List, pub_::Pub},
};

mod echo;
mod hz;
mod info;
mod list;
mod pub_;
pub(crate) mod selector;

/// Manage and inspect dataflow topics.
#[derive(Debug, clap::Subcommand)]
pub enum Topic {
    List(List),
    Echo(Echo),
    Hz(Hz),
    Info(Info),
    Pub(Pub),
}

impl Executable for Topic {
    fn execute(self) -> eyre::Result<()> {
        match self {
            Topic::List(cmd) => cmd.execute(),
            Topic::Echo(cmd) => cmd.execute(),
            Topic::Hz(cmd) => cmd.execute(),
            Topic::Info(cmd) => cmd.execute(),
            Topic::Pub(cmd) => cmd.execute(),
        }
    }
}

/// Run `f`, turning a panic inside arrow into the error `"{what}: {reason}"`.
fn catch_arrow_panic<T>(what: &str, f: impl FnOnce() -> eyre::Result<T>) -> eyre::Result<T> {
    std::panic::catch_unwind(std::panic::AssertUnwindSafe(f)).unwrap_or_else(|panic| {
        let reason = panic
            .downcast_ref::<&str>()
            .copied()
            .or_else(|| panic.downcast_ref::<String>().map(String::as_str))
            .unwrap_or("unknown reason");
        Err(eyre::eyre!("{what}: {reason}"))
    })
}
