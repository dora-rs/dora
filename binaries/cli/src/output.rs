use colored::Colorize;
use dora_core::build::LogLevelOrStdout;
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
        LogLevelOrStdout::LogLevel(level) => match level {
            log::Level::Error => "ERROR ".red(),
            log::Level::Warn => "WARN  ".yellow(),
            log::Level::Info => "INFO  ".green(),
            log::Level::Debug => "DEBUG ".bright_blue(),
            log::Level::Trace => "TRACE ".dimmed(),
        },
        LogLevelOrStdout::Stdout => "stdout".bright_blue().italic().dimmed(),
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
