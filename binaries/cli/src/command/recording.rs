mod diff;
mod export;

use std::io::Read;

use dora_recording::RecordingReader;

use crate::command::Executable;

/// Manage dataflow recordings.
#[derive(Debug, clap::Subcommand)]
pub enum Recording {
    /// Export a recording to an MCAP file
    Export(export::Export),
    /// Compare the messages of two recordings, output by output
    Diff(diff::Diff),
}

impl Executable for Recording {
    fn execute(self) -> eyre::Result<()> {
        match self {
            Recording::Export(cmd) => cmd.execute(),
            Recording::Diff(cmd) => cmd.execute(),
        }
    }
}

/// Warn when `reader` stopped before the footer `RecordingWriter::finish`
/// writes. The reader ends an unfinished recording (a crash or a second
/// Ctrl-C) early without an error, so without this a command that read only
/// part of it would report success as if it had read all of it.
///
/// Call it after the read loop has returned `None`.
pub(crate) fn warn_if_unfinished<R: Read>(reader: &RecordingReader<R>, path: &str) {
    if !reader.reached_footer() {
        eprintln!(
            "warning: recording `{path}` has no footer: it was not finished cleanly \
             (a crash or a second Ctrl-C) and may be missing its last messages"
        );
    }
}
