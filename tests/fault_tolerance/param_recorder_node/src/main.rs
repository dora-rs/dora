//! Test-only node that appends every param event it receives to
//! `$DORA_TEST_MARKER_FILE`: `pid <pid>` on start, then `set <key> <json>` or
//! `del <key>`, one per line, each written with a single `write_all`.

use std::io::Write;

use dora_node_api::{DoraNode, Event};
use eyre::Context;

fn main() -> eyre::Result<()> {
    let (_node, mut events) =
        DoraNode::init_from_env().context("failed to init dora node from env")?;
    let path = std::env::var("DORA_TEST_MARKER_FILE")
        .context("DORA_TEST_MARKER_FILE env var must be set")?;
    let mut file = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(&path)
        .with_context(|| format!("failed to open {path:?}"))?;
    write_line(&mut file, format!("pid {}", std::process::id()))?;

    while let Some(event) = events.recv() {
        let line = match event {
            Event::ParamUpdate { key, value } => format!("set {key} {value}"),
            Event::ParamDeleted { key } => format!("del {key}"),
            Event::Stop(_) => break,
            _ => continue,
        };
        write_line(&mut file, line)?;
    }
    Ok(())
}

/// Writes `line` and its newline in one `write_all`. `writeln!` on an
/// unbuffered `File` issues one `write` per piece, so the test, which polls
/// the file while the node writes, could read half a line.
fn write_line(file: &mut std::fs::File, mut line: String) -> std::io::Result<()> {
    line.push('\n');
    file.write_all(line.as_bytes())
}
