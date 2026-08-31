use eyre::Context;
use std::io::Write;
use std::time::Duration;

fn main() -> eyre::Result<()> {
    let marker_path = std::env::var("DORA_TEST_MARKER_FILE")
        .context("DORA_TEST_MARKER_FILE env var must be set by the fixture")?;
    let mut marker = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(&marker_path)
        .with_context(|| format!("failed to open marker file {marker_path:?}"))?;
    marker
        .write_all(b"incarnation\n")
        .context("failed to append incarnation marker")?;
    drop(marker);

    // Deliberately sleep/block without ever calling DoraNode::init_from_env()
    // or connecting to the daemon.
    loop {
        std::thread::sleep(Duration::from_secs(60));
    }
}
