use adora_node_api::{AdoraNode, Event};
use eyre::{Context, Result};
use std::{
    env,
    fs::OpenOptions,
    io::Write,
};

fn main() -> Result<()> {
    let (_node, mut events) = AdoraNode::init_from_env()?;

    let path = env::var("LOG_FILE").unwrap_or_else(|_| "./combined.jsonl".to_string());
    let mut file = OpenOptions::new()
        .create(true)
        .append(true)
        .open(&path)
        .wrap_err_with(|| format!("failed to open log file: {path}"))?;

    eprintln!("writing combined logs to {path}");
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
                file.write_all(line.as_bytes())
                    .wrap_err("failed to write to log file")?;

                count += 1;
                if count % 100 == 0 {
                    file.flush().wrap_err("failed to flush log file")?;
                }
            }
            Event::Stop(_) => {
                file.flush().wrap_err("failed to flush log file")?;
                eprintln!("file sink stopping, wrote {count} entries to {path}");
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
