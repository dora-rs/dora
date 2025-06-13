use colored::Colorize;
use dora_message::common::LogMessage;

pub fn print_log_message(log_message: LogMessage) {
    let LogMessage {
        build_id: _,
        dataflow_id,
        node_id,
        daemon_id,
        level,
        target,
        module_path: _,
        file: _,
        line: _,
        message,
    } = log_message;
    let level = match level {
        log::Level::Error => "ERROR".red(),
        log::Level::Warn => "WARN ".yellow(),
        log::Level::Info => "INFO ".green(),
        other => format!("{other:5}").normal(),
    };
    let dataflow = if let Some(dataflow_id) = dataflow_id {
        format!(" dataflow `{dataflow_id}`\t").cyan()
    } else {
        String::new().cyan()
    };
    let daemon = match daemon_id {
        Some(id) => match id.machine_id() {
            Some(machine_id) => format!(" on daemon `{machine_id}`\t"),
            None => " on default daemon\t".to_string(),
        },
        None => " on default daemon\t".to_string(),
    }
    .bright_black();
    let node = match node_id {
        Some(node_id) => format!(" {node_id}\t").bold(),
        None => "".normal(),
    };
    let target = match target {
        Some(target) => format!(" {target}\t").dimmed(),
        None => "".normal(),
    };

    println!("{level}\t{dataflow}{daemon}{node}{target}: {message}");
}
