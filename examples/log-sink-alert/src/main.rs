use dora_arrow_convert::IntoArrow;
use dora_message::common::{LogLevel, LogLevelOrStdout};
use dora_node_api::{DoraNode, Event};
use eyre::Result;

fn main() -> Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;

    while let Some(event) = events.recv() {
        match event {
            Event::Input {
                id: _,
                metadata: _,
                data,
            } => {
                let log = match dora_log_utils::parse_log_from_arrow(&data) {
                    Ok(log) => log,
                    Err(e) => {
                        eprintln!("failed to parse log entry: {e}");
                        continue;
                    }
                };

                let json = dora_log_utils::format_json(&log);
                let arrow = json.as_str().into_arrow();

                // Forward error/warn logs to the "alerts" output first to avoid clone.
                let is_alert = matches!(
                    &log.level,
                    LogLevelOrStdout::LogLevel(LogLevel::Error | LogLevel::Warn)
                );
                if is_alert {
                    node.send_output("alerts".into(), Default::default(), arrow.clone())?;
                }

                // Forward all logs to the "all_logs" output.
                node.send_output("all_logs".into(), Default::default(), arrow)?;
            }
            Event::Stop(_) => break,
            Event::InputClosed { id } => {
                eprintln!("input `{id}` closed");
            }
            _ => {}
        }
    }

    Ok(())
}
