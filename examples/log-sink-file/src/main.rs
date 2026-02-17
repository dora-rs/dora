use adora_node_api::{AdoraNode, Event};
use eyre::{Context, Result, bail};
use std::{
    env,
    fs::OpenOptions,
    io::{BufWriter, Write},
    path::{Component, PathBuf},
};

fn main() -> Result<()> {
    let (_node, mut events) = AdoraNode::init_from_env()?;

    let raw = env::var("LOG_FILE").unwrap_or_else(|_| "./combined.jsonl".to_string());
    let path = validate_log_path(&raw)?;

    let file = OpenOptions::new()
        .create(true)
        .append(true)
        .open(&path)
        .wrap_err_with(|| format!("failed to open log file: {}", path.display()))?;
    let mut writer = BufWriter::new(file);

    eprintln!("writing combined logs to {}", path.display());
    let mut count: u64 = 0;

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id: _, metadata: _, data } => {
                let log = match adora_log_utils::parse_log_from_arrow(&data) {
                    Ok(log) => log,
                    Err(e) => {
                        eprintln!("failed to parse log entry: {e}");
                        continue;
                    }
                };

                let line = format!("{}\n", adora_log_utils::format_json(&log));
                writer.write_all(line.as_bytes())
                    .wrap_err("failed to write to log file")?;

                count += 1;
                if count % 100 == 0 {
                    writer.flush().wrap_err("failed to flush log file")?;
                }
            }
            Event::Stop(_) => {
                writer.flush().wrap_err("failed to flush log file")?;
                eprintln!("file sink stopping, wrote {count} entries");
                break;
            }
            Event::InputClosed { id } => {
                eprintln!("input `{id}` closed");
            }
            _ => {}
        }
    }

    Ok(())
}

/// Validate that LOG_FILE is a relative path with no parent-traversal.
fn validate_log_path(raw: &str) -> Result<PathBuf> {
    let path = PathBuf::from(raw);
    if path.is_absolute() {
        bail!("LOG_FILE must be a relative path, got: {raw}");
    }
    if path.components().any(|c| c == Component::ParentDir) {
        bail!("LOG_FILE must not contain '..': {raw}");
    }
    Ok(path)
}
