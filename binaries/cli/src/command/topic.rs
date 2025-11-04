use crate::command::{
    Executable,
    topic::{echo::Echo, hz::Hz, list::List},
};

mod echo;
mod hz;
mod list;
mod selector;

/// Manage and inspect dataflow topics.
///
/// This group contains subcommands to list topics, inspect (echo) topic
/// messages, and measure publish frequency (hz). Common flags such as
/// `-d, --dataflow <UUID_OR_NAME>`, `--coordinator-addr <IP>` and
/// `--coordinator-port <PORT>` are provided by the selector arguments and
/// apply to all subcommands.
///
/// Subcommands:
/// - `list` — List topics for a dataflow.
/// - `echo` — Print topic data to the terminal.
/// - `hz` — Show publishing frequency (Hz) for topics.
///
/// Examples:
/// - List topics: `dora topic list -d my-dataflow`
/// - Echo a single topic: `dora topic echo -d my-dataflow robot1/pose`
/// - Echo all topics: `dora topic echo -d my-dataflow`
/// - Show hz for specific topics: `dora topic hz -d my-dataflow robot1/pose robot2/vel --window 5`
///
/// Note: The dataflow must enable
/// `_unstable_debug.publish_all_messages_to_zenoh: true` in its descriptor in
/// order for `echo` and `hz` to inspect runtime messages. See
/// `topic selector` flags for how to select specific topics (`node_id/output_id`).
#[derive(Debug, clap::Subcommand)]
pub enum Topic {
    List(List),
    Echo(Echo),
    Hz(Hz),
}

impl Executable for Topic {
    fn execute(self) -> eyre::Result<()> {
        match self {
            Topic::List(cmd) => cmd.execute(),
            Topic::Echo(cmd) => cmd.execute(),
            Topic::Hz(cmd) => cmd.execute(),
        }
    }
}
